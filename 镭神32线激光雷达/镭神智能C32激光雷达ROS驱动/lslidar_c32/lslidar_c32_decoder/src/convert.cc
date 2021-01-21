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

#include "lslidar_c32_decoder/convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace lslidar_c32_decoder
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new lslidar_rawdata::RawData())
{
  scan_nums = 0;
  scan_start = true;
  scan_recv.reset(new lslidar_c32_msgs::LslidarC32ScanUnified);
  scan_recv->packets.resize(1000);

  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("LSC32"));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  private_nh.param("output_points_topic", output_points_topic, std::string("lslidar_point_cloud"));
  output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);
  scan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 100);
  
  // subscribe to lslidar packets
  std::string input_packets_topic;
  private_nh.param("input_packets_topic", input_packets_topic, std::string("lslidar_packet"));
  packet_sub_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));

  private_nh.param("time_synchronization", time_synchronization_, false);
  private_nh.param("scan_num", scan_num, 1);
  private_nh.param("publish_scan", publish_scan, false);
  private_nh.param<std::string>("scan_frame_id", scan_frame_id, "laser_link");

	  if (time_synchronization_)
	  {
		sync_sub_ = node.subscribe("sync_header", 10, &Convert::timeSync, (Convert*)this,
					   ros::TransportHints().tcpNoDelay(true));
	  }
	  
	sweep_data = lslidar_c32_msgs::LslidarC32SweepPtr(
                    new lslidar_c32_msgs::LslidarC32Sweep());

}

void Convert::timeSync(const sensor_msgs::TimeReferenceConstPtr &time_msg)
{
  global_time = time_msg->header.stamp;
}

void Convert::publishScan(lslidar_c32_msgs::LslidarC32SweepPtr& sweep_data, int scan_num)
{
    sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);

    if(sweep_data->scans[scan_num].points.size() <= 1)
        return;
	
	uint16_t point_num = 2000;  //65000/32
	double angle_base = M_PI*2 / point_num;

    scan->header.frame_id = scan_frame_id;
    scan->header.stamp = sweep_data->header.stamp;  // timestamp will obtained from sweep data stamp

   scan->time_increment = 0.05;  //s 
    scan->scan_time =1/10.0;  //s
    scan->angle_min = M_PI/2;
    scan->angle_max = M_PI*5/2;
    scan->angle_increment = (scan->angle_max - scan->angle_min)/point_num;
	
    scan->range_min = 0.15;
    scan->range_max = 150;
    scan->ranges.reserve(point_num);
    scan->ranges.assign(point_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(point_num);
    scan->intensities.assign(point_num, std::numeric_limits<float>::infinity());
		
  for(uint16_t i = 0; i < sweep_data->scans[scan_num].points.size()-1; i++)
  {
	int point_idx = sweep_data->scans[scan_num].points[i].azimuth / angle_base;

	if (point_idx >= point_num)
	  point_idx = 0;
	if (point_idx < 0)
	  point_idx = point_num - 1;

	scan->ranges[point_num - 1-point_idx] = sweep_data->scans[scan_num].points[i].distance;
	scan->intensities[point_num - 1-point_idx] = sweep_data->scans[scan_num].points[i].intensity;
  }
    scan_pub.publish(scan);
}


/** @brief Callback for raw scan messages. */
void Convert::processScan(const lslidar_c32_msgs::LslidarC32ScanUnified::ConstPtr& scanMsg)
{
  lslidar_rawdata::VPointCloud::Ptr outPoints(new lslidar_rawdata::VPointCloud);
  sweep_data = lslidar_c32_msgs::LslidarC32SweepPtr(
                    new lslidar_c32_msgs::LslidarC32Sweep());

  //if (time_synchronization_)
  //{
    outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    sweep_data->header.stamp = scanMsg->header.stamp;
  /*}
  else
  {
      outPoints->header.stamp = ros::Time::now().toNSec() / 1000ull;
	   sweep_data->header.stamp = ros::Time::now();
  }*/	
		
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  outPoints->height = 32;
  outPoints->width = 12 * (int)scanMsg->packets.size();
  //ROS_INFO("packets.size=%d",scanMsg->packets.size());
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);
  
  // process each packet provided by the driver
  data_->block_num = 0;
  for (int i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints, sweep_data, i);
  }
  
  if(scan_num < 0)  scan_num = 0;
  else if (scan_num > 31)  scan_num = 31;
	  
  if(publish_scan)
	publishScan(sweep_data, scan_num);
					
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);
  output_.publish(outMsg);
}
}  // namespace lslidar_c32_decoder
