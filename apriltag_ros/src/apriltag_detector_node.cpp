#include "apriltag_ros/apriltag_detector_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), tag_nh_(pnh, "apriltag"), it_(pnh), cfg_server_(tag_nh_) {
  sub_camera_ =
      it_.subscribeCamera("image", 1, &ApriltagDetectorNode::CameraCb, this);
}

void ApriltagDetectorNode::CameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::imshow("image", image);
  cv::waitKey(1);
}

}  // namespace apriltag_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  ros::NodeHandle pnh("~");
  apriltag_ros::ApriltagDetectorNode node(pnh);
  ros::spin();
}
