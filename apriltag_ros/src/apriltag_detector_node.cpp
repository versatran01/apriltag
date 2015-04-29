#include "apriltag_ros/apriltag_detector_node.h"

#include <sensor_msgs/image_encodings.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sv_base/timer.hpp>

#include <geometry_msgs/PoseArray.h>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh), cfg_server_(pnh) {
  pnh_.param("size", tag_size_, 0.0);

  sub_camera_ =
      it_.subscribeCamera("image", 1, &ApriltagDetectorNode::cameraCb, this);
  pub_apriltags_ =
      pnh_.advertise<apriltag_msgs::ApriltagArrayStamped>("apriltags", 1);
  pub_image_ = it_.advertise("image_detection", 1);
  pub_pose_array_ =
      pnh_.advertise<geometry_msgs::PoseArray>("apritlags_pose", 1);
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::configCb, this, _1, _2));
}

void ApriltagDetectorNode::cameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  const auto gray = cv_bridge::toCvShare(
                        image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Detection
  static sv::base::TimerMs timer("detect", false);
  timer.start();
  detector_->detect(gray);
  timer.stop();
  if (timer.count() % 100 == 0) {
    timer.reportStats();
  }

  cv::Mat disp;
  cv::cvtColor(gray, disp, CV_GRAY2BGR);
  detector_->draw(disp);

  // Only estimate if camera info is valid
  if (cinfo_msg->K[0] != 0 && cinfo_msg->height != 0 && tag_size_ > 0) {
    const auto P = cinfo_msg->P;
    cv::Matx33d K(P[0], P[1], P[2], P[4], P[5], P[6], P[8], P[9], P[10]);
    detector_->estimate(K);
  }

  auto apriltag_array_msg =
      boost::make_shared<apriltag_msgs::ApriltagArrayStamped>();
  apriltag_array_msg->header = image_msg->header;
  apriltag_array_msg->apriltags = detector_->toApriltagMsg();
  pub_apriltags_.publish(apriltag_array_msg);

  // Publish detection image if needed
  if (pub_image_.getNumSubscribers() > 0) {
    cv_bridge::CvImage cv_img(image_msg->header,
                              sensor_msgs::image_encodings::BGR8, disp);
    pub_image_.publish(cv_img.toImageMsg());
  }

  geometry_msgs::PoseArray pose_array;
  for (const apriltag_msgs::Apriltag& apriltag :
       apriltag_array_msg->apriltags) {
    pose_array.header = apriltag_array_msg->header;
    pose_array.poses.push_back(apriltag.pose);
  }
  pub_pose_array_.publish(pose_array);

  cv::imshow("image", disp);
  cv::waitKey(1);
}

void ApriltagDetectorNode::configCb(ConfigT& config, int level) {
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

    detector_ = ApriltagDetector::create(detector_type, tag_family);
    detector_->set_tag_size(tag_size_);
    ROS_INFO("Tag size: %f", tag_size_);
  }
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
