cmake_minimum_required(VERSION 2.8.3)
project(robotec_odometry_publisher)

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
    fobots_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotec_odometry_publisher
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs fobots_msgs geometry_msgs sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robotec_odometry_publisher src/odom_publisher.cpp)
add_executable(robotec_odometry_publisher_node src/odom_publisher_node.cpp)
target_link_libraries(robotec_odometry_publisher_node odom_publisher ${catkin_LIBRARIES})
add_dependencies(robotec_odometry_publisher fobots_msgs_generate_messages_cpp)