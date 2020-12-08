#!/bin/bash
# set up ROS
if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
    source /opt/ros/noetic/setup.bash
fi
if [[ -f "/opt/ros/kinetic/setup.bash" ]]; then
    source /opt/ros/kinetic/setup.bash
fi
if [[ -f "/opt/ros/melodic/setup.bash" ]]; then
    source /opt/ros/melodic/setup.bash
fi

# make workspace
mkdir -p ../catkin_ws/src
cd ../catkin_ws

# link source into catkin workspace
ln -s $src_dir src/apriltag

# configure catkin workspace
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

# build it!
catkin build
