cmake_minimum_required(VERSION 3.0.2)
project(pwm_test_pub)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin 
  REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    pwm_test_cpp_pub
  CATKIN_DEPENDS 
    roscpp
    std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# add libraries
add_library(pwm_test_cpp_pub
       src/pwm_test_cpp_pub.cpp
)
add_dependencies(pwm_test_cpp_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pwm_test_cpp_pub
  ${catkin_LIBRARIES}
)


# pwm_test_cpp_pub_node executable
add_executable(pwm_test_cpp_pub_node nodes/pwm_test_cpp_pub_node.cpp)
add_dependencies(pwm_test_cpp_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pwm_test_cpp_pub_node   
  pwm_test_cpp_pub
  ${catkin_LIBRARIES}
)

# python
catkin_install_python(PROGRAMS nodes/pwm_test_python_pub_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS pwm_test_cpp_pub 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})