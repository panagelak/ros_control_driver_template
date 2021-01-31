cmake_minimum_required(VERSION 3.0.2)
project(servokit_driver)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)


find_package(catkin 
  REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})