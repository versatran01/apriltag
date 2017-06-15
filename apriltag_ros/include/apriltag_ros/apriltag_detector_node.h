#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "apriltag_ros/apriltag_detector.h"
#include <apriltag_ros/ApriltagDetectorDynConfig.h>

namespace apriltag_ros {

namespace it = image_transport;
namespace dr = dynamic_reconfigure;

class ApriltagDetectorNode {
public:
  using ConfigT = ApriltagDetectorDynConfig;

  explicit ApriltagDetectorNode(const ros::NodeHandle &pnh);
  void ImageCb(const sensor_msgs::ImageConstPtr &image_msg);
  void ConnectCb();
  void ConfigCb(ConfigT &config, int level);

private:
  ros::NodeHandle pnh_;
  it::ImageTransport it_;
  it::Subscriber sub_image_;

  ConfigT config_;
  boost::mutex connect_mutex_;
  dr::Server<ConfigT> cfg_server_;

  ros::Publisher pub_tags_;
  it::Publisher pub_disp_;

  ApriltagDetectorPtr detector_;
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
