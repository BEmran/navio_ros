cmake_minimum_required(VERSION 2.8.3)
project(testbed_navio)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Declare msg to be generated: 
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

## Specify additional locations of header files
include_directories(
  include
  include/lib
  include/lib/Navio
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/navio_interface.cpp
  include/${PROJECT_NAME}/ros_node.cpp
  include/lib/TimeSampling.cpp
  include/lib/Encoder.cpp
  include/lib/Sensors.cpp
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  INCLUDE_DIRS include/lib/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_generation
)

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${PROJECT_NAME}
-lphidget22 ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/lib/Navio/libnavio.a)

add_executable(regulator src/regulator.cpp)
target_link_libraries(regulator ${catkin_LIBRARIES} ${PROJECT_NAME}
-lphidget22 ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/lib/Navio/libnavio.a)
