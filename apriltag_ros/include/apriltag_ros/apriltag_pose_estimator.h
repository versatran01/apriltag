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

class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  double size(){return size_;}
  int id(){return id_;}
  std::string& frame_name(){return frame_name_;}
 private:
  int id_;
  double size_;
  std::string frame_name_;
};

class ApriltagPoseEstimator {
 public:
  explicit ApriltagPoseEstimator(const ros::NodeHandle& pnh);

 private:
  void ApriltagsCb(const apriltag_msgs::ApriltagArrayStampedConstPtr& apriltags_msg);
  void CinfoCb(const sensor_msgs::CameraInfoConstPtr& cinfo_msg);
  void ConnectCb();
  void InitApriltagMap();
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

  ros::NodeHandle pnh_;
  ros::Publisher pub_poses_;
  ros::Subscriber sub_apriltags_, sub_cinfo_;
  tf2_ros::TransformBroadcaster tf2_br_;
  image_geometry::PinholeCameraModel cam_model_;
  std::map<int, std::pair<apriltag_msgs::Apriltag, AprilTagDescription> > map_;
  std::string frame_id_;
  boost::mutex connect_mutex_;
  bool broadcast_tf_;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_POSE_ESTIMATOR_H_
