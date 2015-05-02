#include "apriltag_ros/apriltag_detector_node.h"

#include <sensor_msgs/image_encodings.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sv_base/timer.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

namespace apriltag_ros {

ApriltagDetectorNode::ApriltagDetectorNode(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh), cfg_server_(pnh) {
  pnh_.param("size", tag_size_, 0.0);
  std::string map_file;
  pnh_.param<std::string>("map", map_file, "");
  map_ = loadApriltagMapYaml(map_file);

  sub_camera_ =
      it_.subscribeCamera("image", 1, &ApriltagDetectorNode::cameraCb, this);
  pub_apriltags_ =
      pnh_.advertise<apriltag_msgs::ApriltagArrayStamped>("/apriltags", 1);
  pub_image_ = it_.advertise("/image_detection", 1);
  pub_pose_array_ =
      pnh_.advertise<geometry_msgs::PoseArray>("/apritlags_pose", 1);
  pub_pose_array_map_ =
      pnh_.advertise<geometry_msgs::PoseArray>("/apriltag_map", 1);
  pub_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("/pose", 1);
  cfg_server_.setCallback(
      boost::bind(&ApriltagDetectorNode::configCb, this, _1, _2));
}

void ApriltagDetectorNode::cameraCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  const auto gray = cv_bridge::toCvShare(
                        image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Detection
  static sv::base::TimerMs timer("detect", false);
  timer.start();
  detector_->detect(gray);
  timer.stop();
  if (timer.count() % 100 == 0) {
    timer.reportStats();
  }

  cv::Mat disp;
  cv::cvtColor(gray, disp, CV_GRAY2BGR);
  detector_->draw(disp);

  // Only estimate if camera info is valid
  if (cinfo_msg->K[0] != 0 && cinfo_msg->height != 0 && tag_size_ > 0) {
    const auto P = cinfo_msg->P;
    cv::Matx33d K(P[0], P[1], P[2], P[4], P[5], P[6], P[8], P[9], P[10]);
    detector_->estimate(K);
  }

  auto apriltag_array_msg =
      boost::make_shared<apriltag_msgs::ApriltagArrayStamped>();
  apriltag_array_msg->header = image_msg->header;
  apriltag_array_msg->apriltags = detector_->toApriltagMsg();
  pub_apriltags_.publish(apriltag_array_msg);

  // Publish detection image if needed
  if (pub_image_.getNumSubscribers() > 0) {
    cv_bridge::CvImage cv_img(image_msg->header,
                              sensor_msgs::image_encodings::BGR8, disp);
    pub_image_.publish(cv_img.toImageMsg());
  }

  geometry_msgs::PoseArray pose_array;
  pose_array.header = image_msg->header;
  for (const apriltag_msgs::Apriltag& apriltag :
       apriltag_array_msg->apriltags) {
    pose_array.poses.push_back(apriltag.pose);
  }
  pub_pose_array_.publish(pose_array);

  geometry_msgs::PoseArray pose_array_map;
  pose_array_map.header.frame_id = "map";
  pose_array_map.header.stamp = image_msg->header.stamp;
  for (const auto& tag : map_.tag_map()) {
    const ApriltagMap::Tag3D& tag_3d = tag.second;
    geometry_msgs::Pose tag_pose;
    const ApriltagMap::Tag3D::Matrix34d& corners = tag_3d.corners();
    tag_pose.position.x = corners(0, 0);
    tag_pose.position.y = corners(1, 0);
    tag_pose.position.z = corners(2, 0);
    tag_pose.orientation.w = tag_3d.q().w();
    tag_pose.orientation.x = tag_3d.q().x();
    tag_pose.orientation.y = tag_3d.q().y();
    tag_pose.orientation.z = tag_3d.q().z();
    pose_array_map.poses.push_back(tag_pose);
  }
  pub_pose_array_map_.publish(pose_array_map);

  // Estimate pose
  if (!map_.empty()) {
    const auto qpb = map_.estimatePose(detector_->tag_detections());
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    bool b;
    std::tie(q, p, b) = qpb;
    if (b) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = image_msg->header.stamp;
      pose.pose.position.x = p(0);
      pose.pose.position.y = p(1);
      pose.pose.position.z = p(2);
      pose.pose.orientation.w = q.w();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pub_pose_.publish(pose);
      // Need a tf from map to camera
      geometry_msgs::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = image_msg->header.frame_id;
      transform.transform.translation.x = p(0);
      transform.transform.translation.y = p(1);
      transform.transform.translation.z = p(2);
      transform.transform.rotation = pose.pose.orientation;
      broadcaster_.sendTransform(transform);
    }
  }

  cv::imshow("image", disp);
  cv::waitKey(1);
}

void ApriltagDetectorNode::configCb(ConfigT& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializing reconfigure server");
  }
  if (level < 0 || config_.type != config.type ||
      config_.family != config.family) {
    std::string tag_family;
    if (config.family == 0) {
      tag_family = "t36h11";
    } else if (config.family == 1) {
      tag_family = "t25h9";
    } else if (config.family == 2) {
      tag_family = "t16h5";
    }

    std::string detector_type;
    if (config.type == 0) {
      detector_type = "mit";
    } else if (config.type == 1) {
      detector_type = "umich";
    }

    ROS_INFO("detector_type: %s, tag_family: %s", detector_type.c_str(),
             tag_family.c_str());
    detector_ = ApriltagDetector::create(detector_type, tag_family);
    detector_->set_tag_size(tag_size_);
    ROS_INFO("Tag size: %f", tag_size_);
  }
  ROS_INFO("Configuring detector");
  detector_->set_black_border(config.black_border);
  detector_->set_decimate(config.decimate);
  detector_->set_refine(config.refine);
  config_ = config;
}

}  // namespace apriltag_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_detector_node");
  ros::NodeHandle pnh("~");
  apriltag_ros::ApriltagDetectorNode node(pnh);
  ros::spin();
}
