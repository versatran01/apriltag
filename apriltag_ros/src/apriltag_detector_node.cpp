#include "apriltag_ros/apriltag_detector_node.h"
#include <apriltag_msgs/ApriltagsStamped.h>

#include <cv_bridge/cv_bridge.h>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle &pnh)
    : pnh_(pnh), it_(pnh), cfg_server_(pnh) {
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::ConfigCb, this, _1, _2));
  sub_image_ = it_.subscribe("image", 1, &ApriltagDetectorNode::ImageCb, this);
  pub_apriltags_ =
      pnh_.advertise<apriltag_msgs::ApriltagsStamped>("apriltags", 1);
  pub_detection_ = it_.advertise("image_detection", 1);
}

void ApriltagDetectorNode::ImageCb(
    const sensor_msgs::ImageConstPtr &image_msg) {
  const auto gray = cv_bridge::toCvShare(image_msg, "mono8")->image;

  apriltag_msgs::ApriltagsStamped tag_msg;
  tag_msg.header = image_msg->header;
  tag_msg.tags = detector_.Detect(gray, max_hamming_);
  pub_apriltags_.publish(tag_msg);

  if (pub_detection_.getNumSubscribers()) {
    cv::Mat color;
    cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);
    DrawApriltags(color, tag_msg.tags);
    pub_detection_.publish(
        cv_bridge::CvImage(tag_msg.header, "bgr8", color).toImageMsg());
  }
}

void ApriltagDetectorNode::ConfigCb(ConfigT &config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  max_hamming_ = config.max_hamming;
  detector_.set_decimate(config.decimate);
  detector_.set_nthreads(config.nthreads);
  detector_.set_sigma(config.sigma);

  if (config.tag36h11) {
    detector_.AddFamily(ApriltagFamily::tag36h11);
  } else {
    detector_.RemoveFamily(ApriltagFamily::tag36h11);
  }

  if (config.tag25h9) {
    detector_.AddFamily(ApriltagFamily::tag25h9);
  } else {
    detector_.RemoveFamily(ApriltagFamily::tag25h9);
  }

  if (config.tag16h5) {
    detector_.AddFamily(ApriltagFamily::tag16h5);
  } else {
    detector_.RemoveFamily(ApriltagFamily::tag16h5);
  }
  ROS_INFO(
      "decimate: %d, nthreads: %d, sigma: %f, max_hamming: %d, families: %zu",
      detector_.decimate(), detector_.nthreads(), detector_.sigma(),
      max_hamming_, detector_.NumFamilies());
}

}  // namespace apriltag_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  apriltag_ros::ApriltagDetectorNode node(ros::NodeHandle("~"));
  ros::spin();
}
