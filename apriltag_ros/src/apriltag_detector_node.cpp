#include "apriltag_ros/apriltag_detector_node.h"

#include <sensor_msgs/image_encodings.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), tag_nh_(pnh, "apriltag"), it_(pnh), cfg_server_(tag_nh_) {
  // Handle rectified image
  const auto resolved_topic = pnh.resolveName("image");
  if (resolved_topic.find("rect") != std::string::npos) image_rectified_ = true;
  tag_nh_.param("size", tag_size_, 0.0);

  sub_camera_ =
      it_.subscribeCamera("image", 1, &ApriltagDetectorNode::CameraCb, this);
  pub_apriltags_ =
      pnh_.advertise<apriltag_msgs::ApriltagArrayStamped>("apriltags", 1);
  pub_image_ = it_.advertise("image_detection", 1);
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::ConfigCb, this, _1, _2));
}

void ApriltagDetectorNode::CameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  const auto gray = cv_bridge::toCvShare(
                        image_msg, sensor_msgs::image_encodings::MONO8)->image;

  detector_->Detect(gray);

  cv::Mat disp;
  cv::cvtColor(gray, disp, CV_GRAY2BGR);
  detector_->Draw(disp);

  // Only estimate if camera info is valid
  if (cinfo_msg->K[0] != 0 && cinfo_msg->height != 0) {
    model_.fromCameraInfo(cinfo_msg);
    if (image_rectified_) {
      // Call estimate with projection matrix
    } else {
      // Call estimate with K and D
    }
  }

  apriltag_msgs::ApriltagArrayStampedPtr apriltag_array_msg =
      boost::make_shared<apriltag_msgs::ApriltagArrayStamped>();
  apriltag_array_msg->header = image_msg->header;
  apriltag_array_msg->apriltags = detector_->ToApriltagMsg();
  pub_apriltags_.publish(apriltag_array_msg);

  if (pub_image_.getNumSubscribers() > 0) {
    cv_bridge::CvImage cv_img(image_msg->header,
                              sensor_msgs::image_encodings::BGR8, disp);
    pub_image_.publish(cv_img.toImageMsg());
  }

  cv::imshow("image", disp);
  cv::waitKey(1);
}

void ApriltagDetectorNode::ConfigCb(ConfigT& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  if (config_.type != config.type || config_.family != config.family) {
    std::string tag_family;
    if (config.family == 0) {
      tag_family = "36h11";
    } else if (config.family == 1) {
      tag_family = "25h9";
    } else if (config.family == 2) {
      tag_family = "16h5";
    }

    std::string detector_type;
    if (config.type == 0) {
      detector_type = "mit";
    } else if (config.type == 1) {
      detector_type = "umich";
    }

    detector_ = ApriltagDetector::Create(detector_type, tag_family);
    detector_->set_tag_size(tag_size_);
    ROS_INFO("Tag size: %f", tag_size_);
  }
  detector_->set_decimate(config.decimate);
  detector_->set_refine(config.refine);
  config_ = config;
}

}  // namespace apriltag_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  ros::NodeHandle pnh("~");
  apriltag_ros::ApriltagDetectorNode node(pnh);
  ros::spin();
}
