cmake_minimum_required(VERSION 2.8.3)
project(robotec_frame_publisher)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf2
	  tf2_ros
    nav_msgs
    robotec_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotec_frame_publisher
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs robotec_msgs geometry_msgs sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(frame_publisher src/frame_publisher.cpp)
target_link_libraries(frame_publisher ${catkin_LIBRARIES})
add_dependencies(frame_publisher robotec_msgs_generate_messages_cpp)
