#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <apriltag_ros/ApriltagDetectorDynConfig.h>
#include "apriltag_ros/apriltag_detector.h"

namespace apriltag_ros {

class ApriltagDetectorNode {
 public:
  using ConfigT = ApriltagDetectorDynConfig;

  explicit ApriltagDetectorNode(const ros::NodeHandle &pnh);
  void ImageCb(const sensor_msgs::ImageConstPtr &image_msg);
  void ConfigCb(ConfigT &config, int level);

 private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;

  dynamic_reconfigure::Server<ConfigT> cfg_server_;

  ros::Publisher pub_apriltags_;
  image_transport::Publisher pub_detection_;

  ApriltagDetector detector_;
  int max_hamming_;
};

}  // namespace apriltag_ros
