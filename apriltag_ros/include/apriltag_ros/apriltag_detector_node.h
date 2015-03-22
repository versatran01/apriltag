#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <apriltag_ros/ApriltagDynConfig.h>

namespace apriltag_ros {

class ApriltagDetectorNode {
 public:
  using ConfigT = ApriltagDynConfig;
  ApriltagDetectorNode(const ros::NodeHandle& pnh);

  void CameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void ConnectCb();

  void ConfigCb(ConfigT& config, int level);

 private:
  ros::NodeHandle pnh_, tag_nh_;
  ros::Publisher pub_apriltags_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher sub_camera_;
  dynamic_reconfigure::Server<ConfigT> cfg_server_;
  ConfigT config_;
};

}  // namespace apriltag_ros
#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
