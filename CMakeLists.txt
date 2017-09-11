cmake_minimum_required(VERSION 2.8.3)
project(testbed_navio)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/Navio/Util.cpp
  include/Navio/PWM.cpp
  include/Navio/MPU9250.cpp
  include/Navio/LSM9DS1.cpp
  include/lib/SamplingTime.cpp
  include/${PROJECT_NAME}/navio_interface.cpp
)

## Specify additional locations of header files
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic src/basic.cpp)
target_link_libraries(basic  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(control_gyro src/control_gyro.cpp)
target_link_libraries(control_gyro  ${catkin_LIBRARIES} ${PROJECT_NAME})
