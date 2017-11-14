cmake_minimum_required(VERSION 2.8.3)
project(control_ur3sim)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs control_msgs sensor_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs control_msgs sensor_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(control_ur3sim src/control_ur3sim.cpp)
target_link_libraries(control_ur3sim ${catkin_LIBRARIES})
add_dependencies(control_ur3sim control_ur3sim_generate_messages_cpp)
