# Odometry Publisher Node
The package is dedicated to be a unified frame publisher for dynamic data. It has a simple interface and a straightforward functionality.

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
│   └── robotec_frame_publisher
│       └── frame_publisher.hpp
├── src
│   └── frame_publisher.cpp
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
* `frame_publisher.h` : the main class declaration
* `frame_publisher.cpp` : program functionality

## Algorithm
The package receives data from user-defined topics (velocity and pose) and packs it into messages with a structure for creating navigation frames (specified by the user).

## Parameters
* `from` : parent frame_id
* `to` : child frame_id
* `twist_topic` : velocity data topic
* `pose_topic` : pose data topic

## Credits
* Original project : https://github.com/linorobot/linorobot
