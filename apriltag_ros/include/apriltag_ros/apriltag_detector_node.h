#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_broadcaster.h>

#include <apriltag_ros/ApriltagDetectorDynConfig.h>
#include "apriltag_ros/apriltag_detector.h"
#include "apriltag_ros/apriltag_map.h"

namespace apriltag_ros {

class ApriltagDetectorNode {
 public:
  using ConfigT = ApriltagDetectorDynConfig;

  explicit ApriltagDetectorNode(const ros::NodeHandle& pnh);

  void cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void connectCb();

  void configCb(ConfigT& config, int level);

 private:
  ros::NodeHandle pnh_;
  ros::Publisher pub_apriltags_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Publisher pub_image_;
  dynamic_reconfigure::Server<ConfigT> cfg_server_;
  ConfigT config_;
  double tag_size_;
  ApriltagDetectorPtr detector_;
  ros::Publisher pub_pose_array_, pub_pose_array_map_, pub_pose_array_cam_;
  ros::Publisher pub_pose_;

  ApriltagMap map_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
