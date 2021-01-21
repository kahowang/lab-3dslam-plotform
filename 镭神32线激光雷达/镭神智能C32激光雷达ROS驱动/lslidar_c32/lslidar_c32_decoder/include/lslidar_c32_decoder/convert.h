/*
 * This file is part of lslidar_n301 driver.
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

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "rawdata.h"
#include <sensor_msgs/TimeReference.h>

namespace lslidar_c32_decoder
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

private:

  void processScan(const lslidar_c32_msgs::LslidarC32ScanUnified::ConstPtr& scanMsg);
  void timeSync(const sensor_msgs::TimeReferenceConstPtr& time_msg);
  /// Pointer to dynamic reconfigure service srv_
  
  // Publish scan Data
    void publishScan(lslidar_c32_msgs::LslidarC32SweepPtr& sweep_data, int scan_num);
	
  boost::shared_ptr<lslidar_rawdata::RawData> data_;
  ros::Subscriber packet_sub_;
  ros::Subscriber sync_sub_;
  ros::Time global_time;
  lslidar_c32_msgs::LslidarC32ScanUnifiedPtr scan_recv;
  lslidar_c32_msgs::LslidarC32SweepPtr sweep_data;
  bool scan_start;
  bool publish_scan;
  int scan_num;
  std::string scan_frame_id;
  bool time_synchronization_;
  size_t scan_nums;
  ros::Publisher output_;
  ros::Publisher scan_pub;
};

}  // namespace lslidar_c32_decoder
#endif
