#include "apriltag_ros/apriltag_detector_node.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/lock_guard.hpp>
#include <boost/timer/timer.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

// using apriltag_msgs::ApriltagArrayStamped;

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle &pnh)
    : pnh_(pnh),
      it_(pnh),
      cfg_server_(pnh),
      detector_(ApriltagFamily::tf36h11) {
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::ConfigCb, this, _1, _2));
  sub_image_ = it_.subscribe("image", 1, &ApriltagDetectorNode::ImageCb, this);

  // Setup connect callback
  //  auto connect_cb = boost::bind(&ApriltagDetectorNode::ConnectCb, this);
  //  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //  pub_tags_ =
  //      pnh_.advertise<ApriltagArrayStamped>("tags", 1, connect_cb,
  //      connect_cb);
  //  pub_disp_ = it_.advertise("disp", 1, connect_cb, connect_cb);

  detector_.set_decimate(2);
  detector_.set_nthreads(2);
}

void ApriltagDetectorNode::ImageCb(
    const sensor_msgs::ImageConstPtr &image_msg) {
  const auto gray = cv_bridge::toCvShare(image_msg, "mono8")->image;

  // Detect
  boost::timer::auto_cpu_timer t;

  detector_.Detect(gray);

  // Publish apriltags
  //  auto apriltag_array_msg = boost::make_shared<ApriltagArrayStamped>();
  //  apriltag_array_msg->header = image_msg->header;
  //  apriltag_array_msg->apriltags = apriltags;
  //  pub_tags_.publish(apriltag_array_msg);

  // Publish detection image if needed
  //  if (pub_disp_.getNumSubscribers()) {
  //    cv::Mat disp;
  //    cv::cvtColor(gray, disp, CV_GRAY2BGR);
  //    DrawApriltags(disp, apriltags);
  //    cv_bridge::CvImage cv_img(image_msg->header, image_encodings::BGR8,
  //    disp); pub_disp_.publish(cv_img.toImageMsg());
  //  }
}

void ApriltagDetectorNode::ConfigCb(ConfigT &config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }

  // Save config
  config_ = config;
}

// void ApriltagDetectorNode::ConnectCb() {
//  boost::lock_guard<boost::mutex> lock(connect_mutex_);
//  if (pub_tags_.getNumSubscribers() == 0 &&
//      pub_disp_.getNumSubscribers() == 0) {
//    // Shutting down if nobody subscribes
//    ROS_DEBUG("%s: No subscribers, shutting down",
//    pnh_.getNamespace().c_str()); sub_image_.shutdown();
//  } else if (!sub_image_) {
//    ROS_DEBUG("%s: Resubscribing", pnh_.getNamespace().c_str());
//    sub_image_ =
//        it_.subscribe("image", 1, &ApriltagDetectorNode::ImageCb, this);
//  }
//}

}  // namespace apriltag_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  apriltag_ros::ApriltagDetectorNode node(ros::NodeHandle("~"));
  ros::spin();
}
