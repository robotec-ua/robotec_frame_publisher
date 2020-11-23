cmake_minimum_required(VERSION 2.8.3)
project(odom_publisher)

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
  LIBRARIES odom_publisher
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs fobots_msgs geometry_msgs sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(odom_publisher src/odom_publisher.cpp)
add_executable(odom_publisher_node src/odom_publisher_node.cpp)
target_link_libraries(odom_publisher_node odom_publisher ${catkin_LIBRARIES})
add_dependencies(odom_publisher fobots_msgs_generate_messages_cpp)


#############
## Install ##
#############

# Future work

#############
## Testing ##
#############

# Future work
