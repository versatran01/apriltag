# Apriltag ROS2

This repo has a ROS2 wrapper around two apriltag detection libraries:
- The [MIT C++ detector
  library](http://people.csail.mit.edu/kaess/apriltags/)
- The [UMich Apriltag library V3](http://april.eecs.umich.edu/wiki/index.php/AprilTags)

It publishes a debug image and a message with the detected tags.

## How to build

Clone this repo into the "src" directory of a ROS workspace:
```
cd src
git clone --branch ros2 https://github.com/versatran01/apriltag.git
```

Fetch the missing workspaces
```
cd ..
wstool init src src/apriltag/apriltag_umich/apriltag_umich.rosinstall 
```

Build (note: until the cmake files in the umich repo are fixed,
only building with "Release" will work, or you have to hack this line
there:``set(CMAKE_BUILD_TYPE "Release" CACHE STRING "set the build type")``):
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## How to run

After compiling, source your ROS2 environment, then overlay the new
workspace:
```
source install/setup.bash
```
Run the node (there is also a component launcher if needed):
```
ros2 launch apriltag_ros detector_node.launch.py 
```

## Parameters

Check out the launch file and adjust the following parameters:
- `detector`: values are 0 for MIT detector (slower, lower detection
  performance, but can handle 2-bit wide black borders and tolerates
  black features encroaching on tag as e.g. in calibration boards)
- `tag_family`: values are 0 for 36h11, 1 for 25h9, 2 for 16h5
- `black_border_width`: set this to 1 for regular tags, 2 for
  e.g. calibration boards with double-wide outer black. Only supported
  for MIT detector.
- `decimate`: how many decimation pyramids to run.

You also need to change the image remapping from `/image` to whatever
the correct topic is.

## Troubleshooting

If the tags are not detected,
- is the border of the tag clean? If not, use the MIT detector.
- do the tags have double wide black borders? If yes, use MIT detector
  and set the border width to 2.
- are any images coming in? Check the debug images (/disp)

