#include "apriltag_ros/apriltag_detector_node.h"

#include <sensor_msgs/image_encodings.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh), cfg_server_(pnh) {
  sub_image_ = it_.subscribe("image", 1, &ApriltagDetectorNode::imageCb, this);
  pub_apriltags_ =
      pnh_.advertise<apriltag_msgs::ApriltagArrayStamped>("apriltag_array", 1);
  pub_image_ = it_.advertise("image_detection", 1);
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::configCb, this, _1, _2));
}

void ApriltagDetectorNode::imageCb(
    const sensor_msgs::ImageConstPtr& image_msg) {
  const auto gray = cv_bridge::toCvShare(
                        image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Detection
  detector_->detect(gray);

  cv::Mat disp;
  cv::cvtColor(gray, disp, CV_GRAY2BGR);
  detector_->draw(disp);

  auto apriltag_array_msg =
      boost::make_shared<apriltag_msgs::ApriltagArrayStamped>();
  apriltag_array_msg->header = image_msg->header;
  apriltag_array_msg->apriltags = detector_->apriltags();
  pub_apriltags_.publish(apriltag_array_msg);

  // Publish detection image if needed
  if (pub_image_.getNumSubscribers() > 0) {
    cv_bridge::CvImage cv_img(image_msg->header,
                              sensor_msgs::image_encodings::BGR8, disp);
    pub_image_.publish(cv_img.toImageMsg());
  }

  cv::imshow("image", disp);
  cv::waitKey(1);
}

void ApriltagDetectorNode::configCb(ConfigT& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  if (level < 0 || config_.type != config.type ||
      config_.family != config.family) {
    std::string tag_family;
    if (config.family == 0) {
      tag_family = "t36h11";
    } else if (config.family == 1) {
      tag_family = "t25h9";
    } else if (config.family == 2) {
      tag_family = "t16h5";
    }

    std::string detector_type;
    if (config.type == 0) {
      detector_type = "mit";
    } else if (config.type == 1) {
      detector_type = "umich";
    }

    ROS_INFO("detector_type: %s, tag_family: %s", detector_type.c_str(),
             tag_family.c_str());
    detector_ = ApriltagDetector::create(detector_type, tag_family);
  }
  ROS_INFO("Configuring detector");
  detector_->set_black_border(config.black_border);
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
