#ifndef APRILTAG_POSE_ESTIMATOR_H_
#define APRILTAG_POSE_ESTIMATOR_H_

#include <tf2_ros/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <apriltag_msgs/msg/apriltag_array_stamped.hpp>
#include <apriltag_msgs/msg/apriltag_pose_stamped.hpp>
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

class ApriltagPoseEstimator : public rclcpp::Node {
 public:
  explicit ApriltagPoseEstimator(const rclcpp::NodeOptions &options);

 private:
  void ApriltagsCb(
      const apriltag_msgs::msg::ApriltagArrayStamped &apriltags_msg);
  void CinfoCb(const sensor_msgs::msg::CameraInfo &cinfo_msg);
  void InitApriltagMap();
  std::map<int, AprilTagDescription> parse_tag_descriptions(
      const std::vector<long int> &ids, const std::vector<double> &tag_sizes,
      const std::vector<std::string> &frame_ids);

  rclcpp::Publisher<apriltag_msgs::msg::ApriltagPoseStamped>::SharedPtr
      pub_poses_;
  rclcpp::Subscription<apriltag_msgs::msg::ApriltagArrayStamped>::SharedPtr
      sub_apriltags_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_br_;
  image_geometry::PinholeCameraModel cam_model_;
  std::map<int, std::pair<apriltag_msgs::msg::Apriltag, AprilTagDescription> >
      map_;
  std::string frame_id_;
  boost::mutex connect_mutex_;
};

};  // namespace apriltag_ros

#endif  // APRILTAG_POSE_ESTIMATOR_H_
