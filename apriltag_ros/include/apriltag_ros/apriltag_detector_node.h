#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <apriltag_ros/ApriltagDetectorDynConfig.h>
#include "apriltag_ros/apriltag_detector.h"

namespace apriltag_ros {

class ApriltagDetectorNode {
 public:
  using ConfigT = ApriltagDetectorDynConfig;

  explicit ApriltagDetectorNode(const ros::NodeHandle& pnh);

  void ImageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void ConnectCb();

  void ConfigCb(ConfigT& config, int level);

 private:
  ros::NodeHandle pnh_;
  ros::Publisher pub_apriltags_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_image_;
  dynamic_reconfigure::Server<ConfigT> cfg_server_;
  ConfigT config_;
  ApriltagDetectorPtr detector_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
