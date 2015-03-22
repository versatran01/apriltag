#include "apriltag_ros/apriltag_detector_node.h"

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh), tag_nh_(pnh, "apriltag") {}
}  // namespace apriltag_ros
