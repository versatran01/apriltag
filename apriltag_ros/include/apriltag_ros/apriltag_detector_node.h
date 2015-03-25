#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <apriltag_ros/ApriltagDynConfig.h>
#include "apriltag_ros/apriltag_detector.h"

namespace apriltag_ros {

class ApriltagDetectorNode {
 public:
  using ConfigT = ApriltagDynConfig;
  explicit ApriltagDetectorNode(const ros::NodeHandle& pnh);

  void cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void connectCb();

  void configCb(ConfigT& config, int level);

 private:
  ros::NodeHandle pnh_, tag_nh_;
  ros::Publisher pub_apriltags_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Publisher pub_image_;
  dynamic_reconfigure::Server<ConfigT> cfg_server_;
  image_geometry::PinholeCameraModel model_;
  ConfigT config_;
  double tag_size_;
  bool image_rectified_{false};
  ApriltagDetectorPtr detector_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
