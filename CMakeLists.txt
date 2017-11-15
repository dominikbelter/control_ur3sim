cmake_minimum_required(VERSION 2.8.3)
project(control_ur3sim)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    eigen_conversions
    moveit_ros_planning_interface
    moveit_ros_move_group
)

find_package(Eigen3 REQUIRED)

## Declare ROS messages and services
add_message_files(FILES Num.msg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs control_msgs sensor_msgs)

## Declare a catkin package
catkin_package(
    LIBRARIES
      control_ur3sim_library
    CATKIN_DEPENDS
            roscpp
            message_filters
            sensor_msgs
            std_msgs
            geometry_msgs
            tf
            tf_conversions
            eigen_conversions
            moveit_ros_planning_interface
            moveit_ros_move_group
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(control_ur3sim src/control_ur3sim.cpp)
target_link_libraries(control_ur3sim ${catkin_LIBRARIES})
add_dependencies(control_ur3sim control_ur3sim_generate_messages_cpp)
