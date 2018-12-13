cmake_minimum_required(VERSION 3.5)
project(apriltag_msgs)

find_package(catkin REQUIRED
             COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(DIRECTORY
                  msg
                  FILES
                  Apriltag.msg
                  ApriltagArrayStamped.msg
                  ApriltagPoseStamped.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
