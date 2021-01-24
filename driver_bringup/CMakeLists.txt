cmake_minimum_required(VERSION 2.8.3)
project(driver_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(

)

add_executable(separate_joint_states_pub src/separate_joint_states_pub.cpp)
add_dependencies(separate_joint_states_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(separate_joint_states_pub ${catkin_LIBRARIES})

add_executable(clone_of_gazebo_sub_map_pub src/clone_of_gazebo_sub_map_pub.cpp)
add_dependencies(clone_of_gazebo_sub_map_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(clone_of_gazebo_sub_map_pub ${catkin_LIBRARIES})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY 
    config 
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

