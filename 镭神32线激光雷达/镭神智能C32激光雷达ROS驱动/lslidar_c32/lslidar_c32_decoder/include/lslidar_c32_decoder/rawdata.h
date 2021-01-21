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

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <ros/ros.h>
#include <ros/package.h>
#include <lslidar_c32_msgs/LslidarC32Packet.h>
#include <lslidar_c32_msgs/LslidarC32ScanUnified.h>
#include <lslidar_c32_msgs/LslidarC32Sweep.h>
#include <lslidar_c32_msgs/LslidarC32Point.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <string.h>
#include <cmath>

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

namespace lslidar_rawdata
{
// static const float  ROTATION_SOLUTION_ = 0.18f;  //水平角分辨率 10hz
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

static const float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

static const float DISTANCE_MAX = 200.0f;            /**< meters */
static const float DISTANCE_MIN = 0.2f;              /**< meters */
static const float DISTANCE_UNIT = 0.25f;      /**< meters */
static const float DISTANCE_UNITR = 0.4f;      /**< meters */

static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
static const float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;  //
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for LSC32 support **/
static const int LSC32_FIRINGS_PER_BLOCK = 1;
static const int LSC32_SCANS_PER_FIRING = 32;
static const float LSC32_BLOCK_TDURATION = 100.0f;  // [µs]
static const float LSC32_DSR_TOFFSET = 3.125f;        // [µs]
static const float LSC32_FIRING_TOFFSET = 100.0f;    // [µs]


static const int TEMPERATURE_MIN = 31;

/*Used for calculation of Angle offset*/
static const float LSC32_AZIMUTH_TOFFSET = 12.98f;  /**< meters */
static const float LSC32_DISTANCE_TOFFSET = 4.94f;  /**< meters */

/** \brief Raw LSLIDAR C32 data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
// block
typedef struct raw_block
{
  uint16_t header;  ///< UPPER_BANK or LOWER_BANK
  uint8_t rotation_1;
  uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 96
} raw_block_t;

struct PointXYZITM {
        PCL_ADD_POINT4D
        uint8_t intensity;
        uint8_t ring;
        uint64_t timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
	
/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};


// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude_original_A[32] = {  //1
  -16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
};

static const double scan_altitude_original_A3[32] = {  //1
  -16,0,-15,1,-14,2,-13,3,-12,4,-11,5,-10,6,-9,7,-8,8,-7,
  9,-6,10,-5,11,-4,12,-3,13,-2,14,-1,15
};

static const double scan_altitude_original_C[32] = {  //0.33
  -18,-15,-12,-10,-8,-7,-6,-5,-4,-3.33,-2.66,-3,-2.33,-2,-1.33,-1.66,
  -1,-0.66,0,-0.33,0.33,0.66,1.33,1,1.66,2,3,4,6,8,11,14
};

static const double scan_altitude_original_C3[32] = {  //0.33
  -18,-1,-15,-0.66,-12,-0.33,-10,0,-8,0.33,-7,0.66,-6,1,-5,1.33,-4,1.66,
  -3.33,2,-3,3,-2.66,4,-2.33,6,-2,8,-1.66,11,-1.33,14
};

static const double scan_altitude[32] = {
        -0.2792526803190927, -0.2617993877991494,
        -0.24434609527920614, -0.22689280275926285,
        -0.20943951023931953, -0.19198621771937624,
        -0.17453292519943295, -0.15707963267948966,
        -0.13962634015954636, -0.12217304763960307,
        -0.10471975511965977, -0.08726646259971647,
        -0.06981317007977318, -0.05235987755982988,
        -0.03490658503988659, -0.017453292519943295,
        0                   , 0.017453292519943295,
        0.03490658503988659, 0.05235987755982988,
        0.06981317007977318, 0.08726646259971647,
        0.10471975511965977, 0.12217304763960307,
        0.13962634015954636, 0.15707963267948966,
        0.17453292519943295, 0.19198621771937624,
        0.20943951023931953, 0.22689280275926285,
        0.24434609527920614, 0.2617993877991494
};

static const double scan_altitude2[32] = {
        -0.3141592653589793,-0.20943951023931956,
        -0.13962634015954636,-0.10471975511965978,
        -0.06981317007977318,-0.05235987755982989,
        -0.04066617157146788,-0.023212879051524585,
        -0.017453292519943295,-0.005759586531581287,
        0.005759586531581287,0.017453292519943295,
        0.02897246558310587,0.05235987755982989,
        0.10471975511965978,0.19198621771937624,
        -0.2617993877991494,-0.17453292519943295,
        -0.12217304763960307,-0.08726646259971647,
        -0.05811946409141117,-0.04642575810304917,
        -0.03490658503988659,-0.02897246558310587,
        -0.011519173063162575,0.0,
        0.011519173063162575,0.023212879051524585,
        0.03490658503988659,0.06981317007977318,
        0.13962634015954636,0.24434609527920614
};

static const double cos_scan_altitude[32] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
    std::cos(scan_altitude[16]), std::cos(scan_altitude[17]),
    std::cos(scan_altitude[18]), std::cos(scan_altitude[19]),
    std::cos(scan_altitude[20]), std::cos(scan_altitude[21]),
    std::cos(scan_altitude[22]), std::cos(scan_altitude[23]),
    std::cos(scan_altitude[24]), std::cos(scan_altitude[25]),
    std::cos(scan_altitude[26]), std::cos(scan_altitude[27]),
    std::cos(scan_altitude[28]), std::cos(scan_altitude[29]),
    std::cos(scan_altitude[30]), std::cos(scan_altitude[31]),
};

static const double sin_scan_altitude[32] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
    std::sin(scan_altitude[16]), std::sin(scan_altitude[17]),
    std::sin(scan_altitude[18]), std::sin(scan_altitude[19]),
    std::sin(scan_altitude[20]), std::sin(scan_altitude[21]),
    std::sin(scan_altitude[22]), std::sin(scan_altitude[23]),
    std::sin(scan_altitude[24]), std::sin(scan_altitude[25]),
    std::sin(scan_altitude[26]), std::sin(scan_altitude[27]),
    std::sin(scan_altitude[28]), std::sin(scan_altitude[29]),
    std::sin(scan_altitude[30]), std::sin(scan_altitude[31]),
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw lslidar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
 typedef PointXYZITM VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

/** \brief lslidar data conversion class */
class RawData
{
public:
  RawData();

  ~RawData()
  {
  }

  /*load the cablibrated files: angle, distance, intensity*/
  void loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh);

  /*unpack the UDP packet and opuput PCL PointXYZI type*/
  void unpack(const lslidar_c32_msgs::LslidarC32Packet& pkt, VPointCloud::Ptr pointcloud, lslidar_c32_msgs::LslidarC32SweepPtr& sweep_data, int Packet_num);

  void processDifop(const lslidar_c32_msgs::LslidarC32Packet::ConstPtr& difop_msg);
  ros::Subscriber difop_sub_;
  bool is_init_curve_;
  bool is_init_angle_;
  bool is_init_top_fw_;
  int block_num = 0;
  int intensity_mode_;
  int intensityFactor;
  int rpm_;

private:
  float R1_;
  float R2_;
  bool angle_flag_;
  float start_angle_;
  float end_angle_;
  float max_distance_;
  float min_distance_;
  int dis_resolution_mode_;
  int return_mode_;
  int lslidar_type;
  bool info_print_flag_;
  bool first_start;
  
  double adjust_angle;
  double adjust_angle_two;
  double adjust_angle_three;
  double adjust_angle_four;

  double scan_altitude_A[32];
  double scan_altitude_C[32];

  int degree_mode_;
  float distance_unit_;  /*Unit of distance*/
  bool config_vert_;
  bool config_vert_angle;
  bool print_vert_;
  double cos_scan_altitude_caliration[LSC32_SCANS_PER_FIRING];
  double sin_scan_altitude_caliration[LSC32_SCANS_PER_FIRING];
};

float sin_azimuth_table[ROTATION_MAX_UNITS];
float cos_azimuth_table[ROTATION_MAX_UNITS];

float VERT_ANGLE[32];
float HORI_ANGLE[32];
float aIntensityCal[7][32];
float aIntensityCal_old[1600][32];
bool Curvesis_new = true;
int g_ChannelNum[32][51];
float CurvesRate[32];

float temper = 31.0;
int tempPacketNum = 0;
int numOfLasers = 16;
int TEMPERATURE_RANGE = 40;

}  // namespace lslidar_rawdata
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_rawdata::PointXYZITM,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (uint8_t, intensity, intensity)
                                          (uint8_t, ring, ring)
                                          (double, timestamp, timestamp)
                                          )
#endif  // __RAWDATA_H
