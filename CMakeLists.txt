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
#  include/${PROJECT_NAME}/*.cpp
#  include/lib/*.cpp
#  include/lib/Navio/Navio+/*.cpp
#  include/lib/Navio/Navio2/*.cpp
#  include/lib/Navio/Common/*.cpp
  include/${PROJECT_NAME}/navio_interface.cpp
  include/lib/TimeSampling.cpp
  include/${PROJECT_NAME}/ros_node.cpp

  include/lib/Encoder.cpp
  include/lib/Sensors.cpp
)

#file(GLOB helloworld_SRC
#    "include/${PROJECT_NAME}/*.cpp"
#    "include/lib/*.cpp"
#)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  INCLUDE_DIRS include/lib/
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_generation
)

add_executable(testbed_demo src/testbed_demo.cpp)
target_link_libraries(testbed_demo ${catkin_LIBRARIES} ${PROJECT_NAME}
-lphidget22 ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/lib/Navio/libnavio.a)

#add_executable(testbed src/testbed.cpp)
#target_link_libraries(testbed ${catkin_LIBRARIES} ${PROJECT_NAME}
#-lphidget22 ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/lib/Navio/libnavio.a)

#add_executable(testbed_basic_tracking src/testbed_basic_tracking.cpp)
#target_link_libraries(testbed_basic_tracking ${catkin_LIBRARIES})

#add_executable(testbed_basic_adaptive src/testbed_basic_adaptive.cpp)
#target_link_libraries(testbed_basic_adaptive ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(testbed_full src/testbed_full.cpp)
#target_link_libraries(testbed_full ${catkin_LIBRARIES} ${PROJECT_NAME})
##add_dependencies(basic ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(control_gyro src/control_gyro.cpp)
#target_link_libraries(control_gyro  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(control_tracking src/control_tracking.cpp)
#target_link_libraries(control_tracking  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(demo src/demo.cpp)
#target_link_libraries(demo  ${catkin_LIBRARIES} ${PROJECT_NAME})

