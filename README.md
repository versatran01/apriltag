# apriltag

A collection of apriltag libraries and a ros node.
Note:  The [ROS2 (foxy) branch](https://github.com/versatran01/apriltag/tree/ros2) is no
longer actively supported. Use instead [this package](https://github.com/ros-misc-utilities/apriltag_detector).

## How to build

Clone this repo into the "src" directory of a ROS workspace, including
submodules, and build:
```
catkin config -DCMAKE_BUILD_TYPE=Release
cd src
git clone --recursive https://github.com/versatran01/apriltag.git
catkin build
```

## apriltag_mit

a c++ library from http://people.csail.mit.edu/kaess/apriltags/

## apriltag_umich

a c library from http://april.eecs.umich.edu/wiki/index.php/AprilTags

To decide which detector to use, have a look at
[the performance comparison of UMich vs MIT detectors](docs/performance_comparison.md).

## apriltag_ros

a ros node for detecting apriltag.

Detect apriltags in image topic `~image`
```
roslaunch apriltag_ros apriltag_detector_node.launch camera:=camera
```

Detect apriltags in images
```
rosrun apriltag_ros apriltag_detect image1.png image2.png -t 0 -b 2
```
## apriltag_msgs

apriltag ros messages

## license

This software and any future contributions to it are licensed under
the [Apache License 2.0](LICENSE).

Note: Michael Kaess's code under apriltag_mit is covered by a GPL V2
license.
