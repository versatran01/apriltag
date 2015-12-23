#include "apriltag_ros/apriltag_detector_node.h"

#include <sensor_msgs/image_encodings.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

using namespace sensor_msgs;
using apriltag_msgs::ApriltagArrayStamped;

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), cfg_server_(pnh) {
  image_transport::ImageTransport it(pnh);
  sub_image_ = it.subscribe("image", 1, &ApriltagDetectorNode::ImageCb, this);
  pub_apriltags_ = pnh_.advertise<ApriltagArrayStamped>("apriltag_array", 1);
  pub_image_ = it.advertise("image_detection", 1);
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::ConfigCb, this, _1, _2));
}

void ApriltagDetectorNode::ImageCb(const ImageConstPtr& image_msg) {
  const auto gray =
      cv_bridge::toCvShare(image_msg, image_encodings::MONO8)->image;

  // Detection
  detector_->Detect(gray);

  // Publish apriltag detection
  auto apriltag_array_msg = boost::make_shared<ApriltagArrayStamped>();
  apriltag_array_msg->header = image_msg->header;
  apriltag_array_msg->apriltags = detector_->apriltags();
  pub_apriltags_.publish(apriltag_array_msg);

  // Publish detection image if needed
  if (pub_image_.getNumSubscribers()) {
    cv::Mat disp;
    cv::cvtColor(gray, disp, CV_GRAY2BGR);
    // Draw detection
    detector_->Draw(disp);
    cv_bridge::CvImage cv_img(image_msg->header, image_encodings::BGR8, disp);
    pub_image_.publish(cv_img.toImageMsg());
  }
}

void ApriltagDetectorNode::ConfigCb(ConfigT& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  if (level < 0 || config_.type != config.type ||
      config_.family != config.family) {
    // TODO: This maybe unsafe if someone changes the order in cfg
    detector_ = ApriltagDetector::Create(static_cast<DetectorType>(config.type),
                                         static_cast<TagFamily>(config.family));
  }
  // TODO: Add useful information here
  ROS_INFO("Configuring detector");
  detector_->set_black_border(config.black_border);
  detector_->set_decimate(config.decimate);
  detector_->set_refine(config.refine);
  // Save config
  config_ = config;
}

}  // namespace apriltag_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  ros::NodeHandle pnh("~");

  try {
    apriltag_ros::ApriltagDetectorNode node(pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
