cmake_minimum_required(VERSION 2.8.3)
project(multi_mpu9250)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  sensor_msgs_ext
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs sensor_msgs_ext
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/multi_mpu9250.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 