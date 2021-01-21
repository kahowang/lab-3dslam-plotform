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

#include "lslidar_c32_decoder/rawdata.h"
#include <angles/angles.h>
namespace lslidar_rawdata
{
static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND65 = 1693;  //65000/384
static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND64 = 1700;  //64000/384
RawData::RawData()
{
  this->is_init_angle_ = false;
  this->is_init_curve_ = false;
  this->is_init_top_fw_ = false;
}

void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string model;
  std::string resolution_param;
  adjust_angle = 0;
  adjust_angle_two = 0;
  adjust_angle_three = 0;
  adjust_angle_four = 0;

  private_nh.param("start_angle", start_angle_, float(0));
  private_nh.param("end_angle", end_angle_, float(360));

  if (start_angle_ < 0 || start_angle_ > 360 || end_angle_ < 0 || end_angle_ > 360)
  {
    start_angle_ = 0;
    end_angle_ = 360;
    ROS_INFO_STREAM("start angle and end angle select feature deactivated.");
  }
  else
  {
    ROS_INFO_STREAM("start angle and end angle select feature activated.");
  }

  angle_flag_ = true;
  if (start_angle_ > end_angle_)
  {
    angle_flag_ = false;
    ROS_INFO_STREAM("Start angle is smaller than end angle, not the normal state!");
  }

  //ROS_INFO_STREAM("start_angle: " << start_angle_ << " end_angle: " << end_angle_ << " angle_flag: " << angle_flag_);
  start_angle_ = start_angle_ / 180 * M_PI;
  end_angle_ = end_angle_ / 180 * M_PI;
	
  private_nh.param("max_distance", max_distance_, 200.0f);
  private_nh.param("min_distance", min_distance_, 0.2f);
  private_nh.param("degree_mode", degree_mode_, 0);
  private_nh.param("distance_unit", distance_unit_, 0.25f);
  private_nh.param("config_vert", config_vert_, true);
  private_nh.param("print_vert", print_vert_, true);

  ROS_INFO_STREAM("config_vert : " << config_vert_ << ", print_vert : " << print_vert_);
  ROS_INFO_STREAM("distance threshlod, max: " << max_distance_ << ", min: " << min_distance_);
  ROS_INFO_STREAM("The distance unit is : " << distance_unit_);

  intensity_mode_ = 1;
  info_print_flag_ = false;
  config_vert_angle = false;
  first_start = true;
  
  private_nh.param("model", model, std::string("LSC32"));
  numOfLasers = 16;
  R1_ = 0.04638;
  R2_ = 0.010875;
  lslidar_type = 2;
  intensityFactor = 51;
  rpm_ = 300;
  
  //return mode default
  private_nh.param("return_mode", return_mode_ , 1);

  // receive difop data
  // subscribe to difop lslidar packets, if not right correct data in difop, it will not revise the correct data in the
  // VERT_ANGLE, HORI_ANGLE etc.
  difop_sub_ = node.subscribe("lslidar_packet_difop", 10, &RawData::processDifop, (RawData*)this);
}

void RawData::processDifop(const lslidar_c32_msgs::LslidarC32Packet::ConstPtr& difop_msg)
{
  const uint8_t* data = &difop_msg->data[0];
  bool is_support_dual_return = false;
  int ndegree = 0;
  int startpos = 0;
  double scan_altitude_original_degree33[32];
  double scan_altitude_original_degree1[32];
  
  // check header
  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
  {
    return;
  }
 
  //v3.0 automatic recognition mode,precision,type
  if(first_start){
	  ROS_INFO_STREAM("The lslidar version is:" << (int)data[1202] <<"."<< (int)(data[1203]/16));
	  first_start = false;
  }
  if((data[1202] >= 0x02 && data[1203] >= 0x80) || data[1202] == 0x03)
  {
	   lslidar_type = 3;
	   startpos = 245;
	   //Horizontal correction Angle
	  adjust_angle = (data[186]*256 + data[187]);         //Angle correction A1
	  if(adjust_angle > 32767){
		adjust_angle = adjust_angle - 65535;
	  }

	  adjust_angle_two = (data[190]*256 + data[191]);     //Angle correction A2
	  if(adjust_angle_two > 32767){
		adjust_angle_two = adjust_angle_two - 65535;
	  }

	  adjust_angle_three = (data[188]*256 + data[189]);   //Angle correction A3
	  if(adjust_angle_three > 32767){
		adjust_angle_three = adjust_angle_three - 65535;
	  }

	  adjust_angle_four = (data[192]*256 + data[193]);    //Angle correction A4
	  if(adjust_angle_four > 32767){
		adjust_angle_four = adjust_angle_four - 65535;
	  }
	  memcpy(scan_altitude_original_degree1,scan_altitude_original_A3,32*8);
	  memcpy(scan_altitude_original_degree33,scan_altitude_original_C3,32*8);
	  
	  if(data[185] == 0 || data[185] == 1)
	  {
		  return_mode_ = data[185]+1;
		  if(data[1195] == 0x21 ) 
			 degree_mode_ = 2;
		  else
			 degree_mode_ = 1;
		  
		  distance_unit_ = 0.4f;
		
		  for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++)
		  {
			//均匀1度校准两列
			if(1 == degree_mode_){
			  cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_original_A3[i]));
			  sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_original_A3[i]));
			  scan_altitude_A[i] = scan_altitude_original_A3[i];
			}

			//0.33度校准四列
			if(2 == degree_mode_){
			  cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_original_C3[i]));
			  sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_original_C3[i]));
			  scan_altitude_C[i] = scan_altitude_original_C3[i];
			}
		  }
	  }
	   //ROS_INFO_STREAM("automatic The return_mode_  is : " << return_mode_);
	   //ROS_INFO_STREAM("automatic The degree_mode_  is : " << degree_mode_);
	   //ROS_INFO_STREAM("automatic The distance_unit_  is : " << distance_unit_);
  }
  else{
	  lslidar_type = 2;
	  startpos = 882;
	  //Horizontal correction Angle
	  adjust_angle = (data[34]*256 + data[35]);         /*Angle correction A1*/
	  if(adjust_angle > 32767){
		adjust_angle = adjust_angle - 65535;
	  }

	  adjust_angle_two = (data[42]*256 + data[43]);     /*Angle correction A2*/
	  if(adjust_angle_two > 32767){
		adjust_angle_two = adjust_angle_two - 65535;
	  }

	  adjust_angle_three = (data[66]*256 + data[67]);   /*Angle correction A3*/
	  if(adjust_angle_three > 32767){
		adjust_angle_three = adjust_angle_three - 65535;
	  }

	  adjust_angle_four = (data[68]*256 + data[69]);    /*Angle correction A4*/
	  if(adjust_angle_four > 32767){
		adjust_angle_four = adjust_angle_four - 65535;
	  }
	  
	    //Vertical Angle Calibration for device package
  for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++)
  {
    //均匀1度校准两列
    if(1 == degree_mode_){
      cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_original_A[i]));
      sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_original_A[i]));
      scan_altitude_A[i] = scan_altitude_original_A[i];

    }

    //0.33度校准四列
    if(2 == degree_mode_){
      cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_original_C[i]));
      sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_original_C[i]));
      scan_altitude_C[i] = scan_altitude_original_C[i];
    }
  }
  
	  memcpy(scan_altitude_original_degree1,scan_altitude_original_A,32*8);
	  memcpy(scan_altitude_original_degree33,scan_altitude_original_C,32*8);
  }

  //Vertical Angle parse
  if(config_vert_){
    for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++){
      uint8_t data1 = data[startpos + 2*i];
      uint8_t data2 = data[startpos + 2*i+1];
      int vert_angle = data1*256 + data2;
      if(vert_angle > 32767){
        vert_angle = vert_angle - 65535;
      }

  
      //均匀1度校准两列
      if(1 == degree_mode_){
        scan_altitude_A[i] = (double)(vert_angle * ROTATION_RESOLUTION);
        if(fabs(scan_altitude_original_degree1[i] - scan_altitude_A[i]) > 1.5){
          scan_altitude_A[i] = scan_altitude_original_degree1[i];
        }
        config_vert_angle = true;
      }

      //0.33度校准四列
      if(2 == degree_mode_){
        scan_altitude_C[i] = (double)(vert_angle * ROTATION_RESOLUTION);
        if(fabs(scan_altitude_original_degree33[i] - scan_altitude_C[i]) > 1.5){
          scan_altitude_C[i] = scan_altitude_original_degree33[i];
        }
        config_vert_angle = true;
      }


    }
  }

  // rpm
  if ((data[8]==0x04) && (data[9]==0xB0))
  {
      rpm_ = 1200;
  }
  else if ((data[8]==0x02) && (data[9]==0x58))
  {
      rpm_ = 600;
  }
  else if ((data[8]==0x01) && (data[9]==0x2C))
  {
      rpm_ = 300;
  } else{
	  rpm_ = (data[8]<<8) | data[9];
	  if(rpm_ < 200)  rpm_ = 600;
    //ROS_WARN("Invalid motor rpm!");
  }

  if(print_vert_){
    ROS_INFO("rpm is %d", rpm_);
  }
}


/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const lslidar_c32_msgs::LslidarC32Packet& pkt, VPointCloud::Ptr pointcloud, lslidar_c32_msgs::LslidarC32SweepPtr& sweep_data,  int Packet_num)
{
  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;
  int azimuth_corrected_offset;
  float azimuth_corrected_offset_f;
  uint64_t point_time;
  
  if(config_vert_angle){

    //Vertical Angle Calibration for device package
    for(int i = 0; i < LSC32_SCANS_PER_FIRING; i++)
    {
      //均匀1度校准两列
      if(1 == degree_mode_){
        cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_A[i]));
        sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_A[i]));
        if(print_vert_){
          ROS_INFO("1 degree Channel %d Data  %f", i, scan_altitude_A[i]);
        }
      }

      //0.33度校准四列
      if(2 == degree_mode_){
        cos_scan_altitude_caliration[i] = std::cos(angles::from_degrees(scan_altitude_C[i]));
        sin_scan_altitude_caliration[i] = std::sin(angles::from_degrees(scan_altitude_C[i]));
        if(print_vert_){
          ROS_INFO("0.33 degree Channel %d Data  %f", i, scan_altitude_C[i]);
        }
      }
      config_vert_angle = false;
    }

  }

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[0];
  uint64_t  packet_end_time = pkt.stamp.toNSec();
  for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
  {

    if (UPPER_BANK != raw->blocks[block].header)
    {
      ROS_INFO_STREAM_THROTTLE(180, "skipping LSLIDAR DIFOP packet");
      //ROS_INFO("distance=%d",1111);
      break;
    }
    azimuth = (float)(256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);
    if (2 == return_mode_){

        if (block < (BLOCKS_PER_PACKET - 2))  // 12
        {
          int azi1, azi2;
          azi1 = 256 * raw->blocks[block + 2].rotation_2 + raw->blocks[block + 2].rotation_1;
          azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
          azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
        }
        else
        {
          int azi1, azi2;
          azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
          azi2 = 256 * raw->blocks[block - 2].rotation_2 + raw->blocks[block - 1].rotation_1;
          azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
        }
        azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
        azimuth_diff = azimuth_diff > 36000 ? azimuth_diff - 36000 : azimuth_diff;

    }
    else {

        if (block < (BLOCKS_PER_PACKET - 1))  // 12
        {
          int azi1, azi2;
          azi1 = 256 * raw->blocks[block + 1].rotation_2 + raw->blocks[block + 1].rotation_1;
          azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
          azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
        }
        else
        {
          int azi1, azi2;
          azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
          azi2 = 256 * raw->blocks[block - 1].rotation_2 + raw->blocks[block - 1].rotation_1;
          azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
        }

        azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
        azimuth_diff = azimuth_diff > 36000 ? azimuth_diff - 36000 : azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < LSC32_FIRINGS_PER_BLOCK; firing++)  // 2
    {
      for (int dsr = 0; dsr < LSC32_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 32  3
      {
        azimuth_corrected_f = azimuth + azimuth_diff / LSC32_SCANS_PER_FIRING * dsr;

        azimuth_corrected_f = azimuth_corrected_f < 0 ? azimuth_corrected_f + 36000 : azimuth_corrected_f;
        azimuth_corrected_f = azimuth_corrected_f >36000 ? azimuth_corrected_f - 36000 : azimuth_corrected_f;
		
		int packet_end = 0;
		if(1 == return_mode_)   
			packet_end = ceil(POINTS_ONE_CHANNEL_PER_SECOND65  * 60 / rpm_)* return_mode_* return_mode_;
		else 
			packet_end = ceil(POINTS_ONE_CHANNEL_PER_SECOND64  * 60 / rpm_)* return_mode_* return_mode_;
		
		packet_end = packet_end * 9 / 10;
		
		//remove redundant angle data
		if(Packet_num < 5 &&  azimuth_corrected_f > 18000)  continue;
	    if(Packet_num > packet_end &&  azimuth_corrected_f < 18000)  continue;
			
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;    //The unit is 0.25cm

        // read intensity
        intensity = raw->blocks[block].data[k + 2];

        float distance2 =distance * DISTANCE_RESOLUTION * distance_unit_;

		 //point timestamp
		if(2 == return_mode_){
			point_time = packet_end_time - 100000 * (5 - block/2) - 3125 * (31 - dsr);
		}else{
			point_time = packet_end_time - 100000 * (11 - block) - 3125 * (31 - dsr);
		}
					
        //pcl::PointXYZI point;
		VPoint point;
        if (distance2 > max_distance_ || distance2 < min_distance_)
        {
            point.x = NAN;
            point.y = NAN;
            point.z = NAN;
            point.intensity = 0;
			point.ring = dsr;
			point.timestamp = point_time ;
            pointcloud->at(this->block_num + firing, dsr) = point;
        }
        else
        {
            //均匀1度校准两列
            if(1 == degree_mode_)
            {
              double adjust_diff = adjust_angle_two - adjust_angle;
              if(adjust_diff > 300 && adjust_diff < 460){
					//v2.7 calibrtation
					if(lslidar_type == 3)
					{
						if(1 >= dsr % 4){
						  azimuth_corrected_f += adjust_angle_two;
						}else{
						  azimuth_corrected_f += adjust_angle;
						}
					}
					else
					{
						if(0 == dsr % 2){
						  azimuth_corrected_f += adjust_angle_two;
						}else{
						  azimuth_corrected_f += adjust_angle;
						}
					}
              }else{
                //v2.6 calibrtation
                if(0 == dsr % 2){
                  azimuth_corrected_f += adjust_angle;
                }else{
                  azimuth_corrected_f -= adjust_angle;
                }
              }
            }
			
            //0.33度校准四列
            if(2 == degree_mode_)
            {
              double adjust_diff_one = adjust_angle_two - adjust_angle;
              double adjust_diff_two = adjust_angle_four - adjust_angle_three;
				
				if(lslidar_type == 3)
				{
					//v3.0 calibrtation
					if(0 == dsr || 1 == dsr || 4 == dsr || 8 == dsr || 9 == dsr || 12 == dsr || 16 == dsr || 17 == dsr || 21 == dsr  || 24 == dsr  || 25 == dsr  || 29 == dsr)
					{
						azimuth_corrected_f += adjust_angle_four;   //A4
					}

					if(2 == dsr || 3 == dsr || 6 == dsr || 10 == dsr  || 11 == dsr  || 14 == dsr  || 18 == dsr  || 19 == dsr  || 23 == dsr  || 26 == dsr  || 27 == dsr  || 31 == dsr)
					{
						azimuth_corrected_f += adjust_angle_three;  //A3
					}

					if(5== dsr || 13 == dsr || 20 == dsr || 28 == dsr)
					{
						azimuth_corrected_f += adjust_angle_two;    //A2
					}

					if(7 == dsr || 15 == dsr || 22 == dsr || 30 == dsr)
					{
						azimuth_corrected_f += adjust_angle;        //A1
					}
				}
                else if(adjust_diff_one > 500 && adjust_diff_one < 660 && adjust_diff_two > 150 && adjust_diff_two < 350)
				{
					//v2.7 calibrtation
					if(10 == dsr || 14 == dsr || 18 == dsr || 22 == dsr)
					{
						azimuth_corrected_f += adjust_angle_four;   //A4
					}

					if(11 == dsr || 15 == dsr || 19 == dsr || 23 == dsr)
					{
						azimuth_corrected_f += adjust_angle_three;  //A3
					}

					if(0 == dsr || 2 == dsr || 4 == dsr || 6 == dsr || 8 == dsr || 12 == dsr || 16 == dsr || 20 == dsr || 24 == dsr || 26 == dsr || 28 == dsr || 30 == dsr)
					{
						azimuth_corrected_f += adjust_angle_two;    //A2
					}

					if(1 == dsr || 3 == dsr || 5 == dsr || 7 == dsr || 9 == dsr || 13 == dsr || 17 == dsr || 21 == dsr || 25 == dsr || 27 == dsr || 29 == dsr || 31 == dsr)
					{
						azimuth_corrected_f += adjust_angle;        //A1
					}
					
              }else{

                //v2.6 calibrtation
                if(10 == dsr || 14 == dsr || 18 == dsr || 22 == dsr)
                {
                    azimuth_corrected_f += adjust_angle;
                }

                if(11 == dsr || 15 == dsr || 19 == dsr || 23 == dsr)
                {
                    azimuth_corrected_f -= adjust_angle;
                }

                if(0 == dsr || 2 == dsr || 4 == dsr || 6 == dsr || 8 == dsr || 12 == dsr || 16 == dsr || 20 == dsr || 24 == dsr || 26 == dsr || 28 == dsr || 30 == dsr)
                {
                    azimuth_corrected_f += adjust_angle_two;
                }

                if(1 == dsr || 3 == dsr || 5 == dsr || 7 == dsr || 9 == dsr || 13 == dsr || 17 == dsr || 21 == dsr || 25 == dsr || 27 == dsr || 29 == dsr || 31 == dsr)
                {
                    azimuth_corrected_f -= adjust_angle_two;
                }
              }

            }   
            azimuth_corrected_f = azimuth_corrected_f < 0 ? azimuth_corrected_f + 36000 : azimuth_corrected_f;
            azimuth_corrected_f = azimuth_corrected_f >36000 ? azimuth_corrected_f - 36000 : azimuth_corrected_f;
			
            //以结构为中心
            float rotation_azimuth = angles::from_degrees(ROTATION_RESOLUTION * azimuth_corrected_f);
            azimuth_corrected_offset_f = azimuth_corrected_f*ROTATION_RESOLUTION - LSC32_AZIMUTH_TOFFSET;
            float rotation_azimuth_offset = angles::from_degrees(azimuth_corrected_offset_f);
			
            point.x = distance2 * cos_scan_altitude_caliration[dsr]  * sinf(rotation_azimuth) + (LSC32_DISTANCE_TOFFSET * sinf(rotation_azimuth_offset)) * DISTANCE_RESOLUTION;
            point.y = distance2 * cos_scan_altitude_caliration[dsr]  * cosf(rotation_azimuth) + (LSC32_DISTANCE_TOFFSET * cosf(rotation_azimuth_offset)) * DISTANCE_RESOLUTION;
            point.z = distance2 * sin_scan_altitude_caliration[dsr];
            point.intensity = intensity;
			point.ring = dsr;
			point.timestamp = point_time ;
            pointcloud->at(this->block_num + firing, dsr) = point;
			
			//Save to LaserScan display
			 sweep_data->scans[dsr].points.push_back(
                            lslidar_c32_msgs::LslidarC32Point());
                lslidar_c32_msgs::LslidarC32Point& new_point =
                        sweep_data->scans[dsr].points[
                        sweep_data->scans[dsr].points.size()-1];

                // Pack the data into point msg
                new_point.x = point.x;
                new_point.y = point.y;
                new_point.z = point.z;
                new_point.azimuth = rotation_azimuth;
                new_point.distance = sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));//distance2;
                new_point.intensity = point.intensity;
        }
      }
    }
  }
}


}  // namespace lslidar_c32_decoder
