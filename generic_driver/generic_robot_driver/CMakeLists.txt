cmake_minimum_required(VERSION 2.8.3)
project(generic_robot_driver)

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
  roscpp
  rosparam_shortcuts
  std_msgs
  arm_msgs
  realtime_tools
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    generic_robot_driver
  CATKIN_DEPENDS 
    roscpp
    realtime_tools
    rosparam_shortcuts
    std_msgs
    arm_msgs
  DEPENDS 
    Boost
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

# generic Driver
add_library(generic_robot_driver
  src/generic_robot_driver.cpp
)
add_dependencies(generic_robot_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(generic_robot_driver
  ${catkin_LIBRARIES}
)


install(TARGETS generic_robot_driver
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

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
