cmake_minimum_required(VERSION 2.8.3)
project(multi_mpu6050)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  sensor_msgs_ext
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs sensor_msgs_ext
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(multi_mpu6050_node_cpp src/multi_mpu6050_node.cpp)
target_link_libraries(multi_mpu6050_node_cpp ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/fake_smbus.py
  scripts/mpu_6050.py
  scripts/registers.py
  scripts/multi_mpu6050.py
  scripts/gyro_calibrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 