#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <apriltag_ros/ApriltagDetectorDynConfig.h>
#include "apriltag_ros/apriltag_detector.h"

namespace apriltag_ros {

namespace it = image_transport;
namespace dr = dynamic_reconfigure;

class ApriltagDetectorNode {
 public:
  using ConfigT = ApriltagDetectorDynConfig;

  explicit ApriltagDetectorNode(const ros::NodeHandle& pnh);
  void ImageCb(const sensor_msgs::ImageConstPtr& image_msg);
  void ConnectCb();
  void ConfigCb(ConfigT& config, int level);

 private:
  ros::NodeHandle pnh_;
  it::ImageTransport it_;
  it::Subscriber sub_image_;
  ros::Publisher pub_apriltags_;
  it::Publisher pub_detection_;
  dr::Server<ConfigT> cfg_server_;
  ConfigT config_;
  ApriltagDetectorPtr detector_;
  boost::mutex connect_mutex_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
