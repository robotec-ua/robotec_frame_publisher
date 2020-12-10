# Odometry Publisher Node
The package is created for only one task : to be a middle-man between odometry data source and EKF and produce only one type of messages, used to create navigation frames (map and odom).

## Table of contents
* [Structure](#project_structure)
    * [Folders](#folders)
    * [Files](#files)
* [Algorithm](#algorithm)
* [Parameters](#parameters)
* [Credits](#credits)

## Project structure
```
.
├── include
│   └── odometry_publisher
│       └── odom_publisher.h
├── src
│   ├── odom_publisher.cpp
│   └── odom_publisher_node.cpp
├── CMakeLists.txt
├── LICENSE
├── package.xml
└── README.md
```

### Folders
* `include` : C/C++ Headers
* `src` : code of the package

### Files
* `README.md` : the documentation of the project
* `odom_publisher.h` : the main class declaration
* `odom_publisher.cpp` : program functionality
* `odom_publisher_node.cpp` : node declaration

## Algorithm
The package receives data from user-defined topics (velocity) and packs it into messages with a structure for creating navigation frames (map and odom) by EKF.

## Parameters
* `odom_topic` : topic where to push odometry data, default : `raw_odom`
* `velocity_topic` : velocity data, default : `raw_vel`
* `setpose_service` : service for setting the pose of a robot, default : `set_pose`

## Credits
* Original project : https://github.com/linorobot/linorobot
