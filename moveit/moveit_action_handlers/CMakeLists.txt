cmake_minimum_required(VERSION 3.0.2)
project(moveit_action_handlers)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

# check c++17 / c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
  message(STATUS "Compile with C++17 support")
  add_compile_options(-std=c++17)
elseif(COMPILER_SUPPORTS_CXX11)
message(STATUS "Compile with C++11 support")
  add_compile_options(-std=c++11)  
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "Compile with C++11 support")
  add_compile_options(-std=c++0x)
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++ support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()


find_package(catkin 
  REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  tf2_ros
  tf2_geometry_msgs
  rosparam_shortcuts
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
)

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs  
 )

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    moveit_action_handlers
  CATKIN_DEPENDS 
    actionlib
    actionlib_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    tf2_ros
    tf2_geometry_msgs
    rosparam_shortcuts
    message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

# add libraries
add_library(moveit_action_handlers
       src/move_joints_action_server.cpp
       src/move_topose_action_server.cpp
)
add_dependencies(moveit_action_handlers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})
target_link_libraries(moveit_action_handlers
  ${catkin_LIBRARIES}
)

# move_joints_action_server_node executable
add_executable(move_joints_action_server_node nodes/move_joints_action_server_node.cpp)
add_dependencies(move_joints_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_joints_action_server_node   
  moveit_action_handlers
  ${catkin_LIBRARIES}
)

# move_topose_action_server_node executable
add_executable(move_topose_action_server_node nodes/move_topose_action_server_node.cpp)
add_dependencies(move_topose_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_topose_action_server_node   
  moveit_action_handlers
  ${catkin_LIBRARIES}
)

# move joints test arm client
add_executable(move_joints_arm_test_client cpp_test_clients/move_joints_arm_test_client.cpp)
add_dependencies(move_joints_arm_test_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})
target_link_libraries(move_joints_arm_test_client   
  ${catkin_LIBRARIES}
)

# move joints test gripper client
add_executable(move_joints_gripper_test_client cpp_test_clients/move_joints_gripper_test_client.cpp)
add_dependencies(move_joints_gripper_test_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})
target_link_libraries(move_joints_gripper_test_client   
  ${catkin_LIBRARIES}
)

# move to pose arm test client
add_executable(move_topose_arm_test_client cpp_test_clients/move_topose_arm_test_client.cpp)
add_dependencies(move_topose_arm_test_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_generate_messages_cpp})
target_link_libraries(move_topose_arm_test_client   
  ${catkin_LIBRARIES}
)

# python clients
catkin_install_python(PROGRAMS 
    python_test_clients/move_joints_arm_test_client.py
    python_test_clients/move_joints_gripper_test_client.py
    python_test_clients/move_topose_arm_test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS moveit_action_handlers 
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

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})