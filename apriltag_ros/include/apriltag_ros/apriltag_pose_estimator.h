#ifndef APRILTAG_POSE_ESTIMATOR_H_
#define APRILTAG_POSE_ESTIMATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <apriltag_msgs/ApriltagPoseStamped.h>
#include <boost/thread/mutex.hpp>

namespace apriltag_ros {

namespace am = apriltag_msgs;
namespace ig = image_geometry;

class ApriltagPoseEstimator {
 public:
  explicit ApriltagPoseEstimator(const ros::NodeHandle& pnh);

  void ApriltagsCb(const am::ApriltagArrayStampedConstPtr& apriltags_msg);
  void InitApriltagMap();
  void CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void ConnectCb();

 private:
  ros::NodeHandle pnh_;
  ros::Publisher pub_poses_;
  ros::Subscriber sub_apriltags_, sub_cinfo_;
  tf2_ros::TransformBroadcaster tf2_br_;
  ig::PinholeCameraModel cam_model_;
  std::map<int, am::Apriltag> map_;
  std::string frame_id_;
  boost::mutex connect_mutex_;
  bool broadcast_tf_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_POSE_ESTIMATOR_H_
