#include "apriltag_ros/apriltag_detector_node.h"

#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), tag_nh_(pnh, "apriltag"), it_(pnh), cfg_server_(tag_nh_) {
  sub_camera_ =
      it_.subscribeCamera("image", 1, &ApriltagDetectorNode::CameraCb, this);
  pub_apriltags_ =
      pnh_.advertise<apriltag_msgs::ApriltagArrayStamped>("apriltags", 1);
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::ConfigCb, this, _1, _2));
}

void ApriltagDetectorNode::CameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  // Only show detection if camera is not calibrated
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->height == 0) {
    ROS_ERROR_THROTTLE(1, "%s: %s", pnh_.getNamespace().c_str(),
                       "camera not calibrated");
    camera_calibrated_ = false;
  }
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  model_.fromCameraInfo(cinfo_msg);

  cv::imshow("image", image);
  cv::waitKey(1);
}

void ApriltagDetectorNode::ConfigCb(ConfigT& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  config_ = config;
}

}  // namespace apriltag_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  ros::NodeHandle pnh("~");
  apriltag_ros::ApriltagDetectorNode node(pnh);
  ros::spin();
}
