cmake_minimum_required(VERSION 2.8.3)
project(axis_camera)


find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  std_msgs
)
catkin_python_setup()

# ROS message generation
add_message_files(FILES axis_ptz_msg.msg axis_ptz_cmd.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package()

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

