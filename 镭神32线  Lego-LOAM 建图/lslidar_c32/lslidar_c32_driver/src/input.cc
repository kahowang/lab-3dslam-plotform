/*
 * This file is part of lslidar_c32 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "lslidar_c32_driver/input.h"

extern volatile sig_atomic_t flag;
namespace lslidar_c32_driver
{
static const size_t packet_size = sizeof(lslidar_c32_msgs::LslidarC32Packet().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
  npkt_update_flag_ = false;
  cur_rpm_ = 0;
  return_mode_ = 1;

  private_nh.param("device_ip", devip_str_, std::string(""));
  private_nh.param<bool>("add_multicast", add_multicast, false);
  private_nh.param<std::string>("group_ip", group_ip, "224.1.1.2");
  if (!devip_str_.empty())
    ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
}

int Input::getRpm(void)
{
  return cur_rpm_;
}

int Input::getReturnMode(void)
{
  return return_mode_;
}

bool Input::getUpdateFlag(void)
{
  return npkt_update_flag_;
}

void Input::clearUpdateFlag(void)
{
  npkt_update_flag_ = false;
}
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port)
{
  timeout_pub = private_nh.advertise<std_msgs::String>("timeout_topic",10);
  msop_timeout = false;
  difop_timeout = false;																				 
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  ROS_INFO_STREAM("Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");  // TODO: ROS_ERROR errno
    return;
  }
	
	if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
			group.imr_interface.s_addr =  htonl(INADDR_ANY);
			//group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                perror("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }
		
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one lslidar packet. */
int InputSocket::getPacket(lslidar_c32_msgs::LslidarC32Packet* pkt, const double time_offset, std::string pkg_name)
{
  double time1 = ros::Time::now().toSec();
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1200;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  while (flag == 1)
 // while (true)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
	  int retval;
	  retval = poll(fds, 1, POLL_TIMEOUT);
		
	  if(retval >0){
		time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
				
		if(msop_timeout){
			std_msgs::String renew_msg;
			renew_msg.data = "0003";
			timeout_pub.publish(renew_msg);
			ROS_WARN(" renew_code: [%s] ,lslidar [%s] renew: %s",renew_msg.data.c_str(), pkg_name.c_str(), bufTime);
			msop_timeout = false;
		}
		if(difop_timeout){
			std_msgs::String renew_msg;
			renew_msg.data = "0004";
			timeout_pub.publish(renew_msg);
			ROS_WARN(" renew_code: [%s] ,lslidar [%s] renew: %s",renew_msg.data.c_str(), pkg_name.c_str(), bufTime);
			difop_timeout = false;
		}
	  }

      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
		time_t curTime = time(NULL);
		struct tm *curTm = localtime(&curTime);
		char bufTime[30] = {0};
		sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
				curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
		
		if(pkg_name == "msop_pkg"){
			std_msgs::String err_msg;
			err_msg.data = "0001";
			timeout_pub.publish(err_msg);
			ROS_WARN(" error_code: [%s] ,lslidar [%s] poll() timeout: %s %s",err_msg.data.c_str(), pkg_name.c_str(), bufTime, devip_str_.c_str());
			msop_timeout = true;
		}
		if(pkg_name == "difop_pkg"){
			std_msgs::String err_msg;
			err_msg.data = "0002";
			timeout_pub.publish(err_msg);
			ROS_WARN(" error_code: [%s] ,lslidar [%s] poll() timeout: %s %s",err_msg.data.c_str(), pkg_name.c_str(), bufTime, devip_str_.c_str());
			difop_timeout = true;
		}  
        //ROS_WARN_THROTTLE(2, "lslidar poll() timeout");
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        ROS_ERROR("poll() reports lslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break;  // done
    }

    ROS_DEBUG_STREAM("incomplete lslidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }

  if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
  {//difop

    int rpm = (pkt->data[8]<<8)|pkt->data[9];
	  // ROS_INFO("rpm=%d",rpm);
    int mode = 1;
    if (cur_rpm_ != rpm || return_mode_ != mode)
    {
      cur_rpm_ = rpm;
      return_mode_ = mode;

      npkt_update_flag_ = true;
    }
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);

  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = NULL;
  empty_ = true;

  // get parameters using private node handle
  private_nh.param("read_once", read_once_, false);
  private_nh.param("read_fast", read_fast_, false);
  private_nh.param("repeat_delay", repeat_delay_, 0.0);

  if (read_once_)
    ROS_INFO("Read input file only once.");
  if (read_fast_)
    ROS_INFO("Read input file as quickly as possible.");
  if (repeat_delay_ > 0.0)
    ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);

  // Open the PCAP dump file
  // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
  ROS_INFO_STREAM("Opening PCAP file " << filename_);
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
  {
    ROS_FATAL("Error opening lslidar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one lslidar packet. */
int InputPCAP::getPacket(lslidar_c32_msgs::LslidarC32Packet* pkt, const double time_offset, std::string pkg_name)
{
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

 // while (flag == 1)
  while (true)
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
        continue;

      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();


      memcpy(&pkt->data[0], pkt_data + 42, packet_size);

      if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
      {//difop
        int rpm = (pkt->data[8]<<8)|pkt->data[9];
        int mode = 1;

        if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08)
            || (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
        {
          if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
          {
            mode = 0;
          }
        }

        if (cur_rpm_ != rpm || return_mode_ != mode)
        {
          cur_rpm_ = rpm;
          return_mode_ = mode;

          npkt_update_flag_ = true;
        }
      }

      pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
                                      // synchronization required
      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
      ROS_INFO("end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    ROS_DEBUG("replaying lslidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again

  if (flag == 0)
  {
    abort();
  }
}
}
