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

#include <ros/ros.h>
#include "lslidar_c32_driver/lslidar_c32_driver.h"
#include "std_msgs/String.h"

using namespace lslidar_c32_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lslidar_c32_driver");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  signal(SIGINT, my_handler);

  // start the driver
  lslidar_c32_driver::lslidarDriver dvr(node, private_nh);
  // loop until shut down or end of file
  while (ros::ok() && dvr.poll())
  {
    ros::spinOnce();
  }

  return 0;
}
