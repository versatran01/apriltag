#include "apriltag_ros/apriltag_pose_estimator.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/thread/lock_guard.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rcpputils/asserts.hpp>


namespace apriltag_ros {

namespace am = apriltag_msgs::msg;
namespace ig = image_geometry;

cv::Mat QuatFromRvec(const cv::Mat &r) {
  // theta = norm(r)
  // q = [cos(theta/2), sin(theta/2) * r / theta]

  cv::Mat q = cv::Mat::zeros(cv::Size(1, 4), CV_64F);
  auto *pq = q.ptr<double>();
  const auto *pr = r.ptr<double>();
  const auto rx = pr[0], ry = pr[1], rz = pr[2];
  const double theta = std::sqrt(rx * rx + ry * ry + rz * rz);
  if (theta < std::numeric_limits<double>::epsilon() * 10.0) {
    pq[0] = 1.0;
    return q;
  }

  const double half_theta = theta * 0.5;
  const double half_sin = std::sin(half_theta);
  const double half_cos = std::cos(half_theta);
  const double k = half_sin / theta;
  pq[0] = half_cos;
  pq[1] = k * rx;
  pq[2] = k * ry;
  pq[3] = k * rz;

  return q;
}

ApriltagPoseEstimator::ApriltagPoseEstimator(
    const rclcpp::NodeOptions &options)
    : Node("tag_detector", options) {

  broadcast_tf_ = declare_parameter("broadcast_tf", false);

  using std::placeholders::_1;
  sub_cinfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS(10).reliable(), std::bind(&ApriltagPoseEstimator::CinfoCb, this, _1) );

  pub_poses_= this->create_publisher<apriltag_msgs::msg::ApriltagPoseStamped>(
    "apriltag_poses", rclcpp::QoS(1));
  RCLCPP_WARN(this->get_logger(), "%s: Subscribing", this->get_namespace());
  sub_apriltags_= this->create_subscription<apriltag_msgs::msg::ApriltagArrayStamped>(
    "apriltags", rclcpp::QoS(1),
    std::bind(&ApriltagPoseEstimator::ApriltagsCb, this, _1));

  InitApriltagMap();
}

void ApriltagPoseEstimator::ApriltagsCb(const am::ApriltagArrayStamped &apriltags_msg) {
  if (!cam_model_.initialized() || map_.empty()) return;

  am::ApriltagPoseStamped apriltag_poses;

  for (const am::Apriltag &apriltag : apriltags_msg.apriltags) {
    auto search = map_.find(apriltag.id);
    if (search != map_.end()) {

      const auto des_pair = search->second;
      const am::Apriltag &map_tag = des_pair.first;

      std::vector<cv::Point2d> img_pts;
      // Add points to img_pts
      for (const auto &c : apriltag.corners) {
        img_pts.emplace_back(c.x, c.y);
      }

      std::vector<cv::Point3d> obj_pts;
      // Add points to obj_pts
      for (const auto &c : map_tag.corners) {
        obj_pts.emplace_back(c.x, c.y, c.z);
      }

      // Check if empty
      if (img_pts.empty()) continue;

      cv::Mat rvec, tvec;
      sensor_msgs::msg::CameraInfo cinfo = cam_model_.cameraInfo();
      if(cinfo.distortion_model == "fisheye") {

        std::vector<cv::Point2d> undistorted_pts;
        cv::fisheye::undistortPoints(img_pts, undistorted_pts, cam_model_.fullIntrinsicMatrix(), cam_model_.distortionCoeffs());

        cv::Mat normalized_cam_mat = cv::Mat::eye(3,3,cv::DataType<double>::type);
        cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);
        auto good = cv::solvePnP(obj_pts, undistorted_pts, normalized_cam_mat, dist_coeffs, rvec, tvec);
        if (!good) {
          RCLCPP_WARN(this->get_logger(), "%s: Pose solver failed.", this->get_namespace());
          continue;
        }
      }
      else {
        // Solve for pose
        // The estiamted r and t brings points from tag frame to camera frame
        // r = c_r_w, t = c_t_w
        auto good = cv::solvePnP(obj_pts, img_pts, cam_model_.fullIntrinsicMatrix(),
                                 cam_model_.distortionCoeffs(), rvec, tvec);
        if (!good) {
          RCLCPP_WARN(this->get_logger(), "%s: Pose solver failed.", this->get_namespace());
          continue;
        }
      }

      cv::Matx33d r;
      cv::Rodrigues(rvec, r);

      tf2::Matrix3x3 wRo(r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2));
      tf2::Quaternion tf_quat;
      wRo.getRotation(tf_quat);

      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = tvec.at<double>(0);
      pose_msg.position.y = tvec.at<double>(1);
      pose_msg.position.z = tvec.at<double>(2);
      pose_msg.orientation.x = tf_quat.x();
      pose_msg.orientation.y = tf_quat.y();
      pose_msg.orientation.z = tf_quat.z();
      pose_msg.orientation.w = tf_quat.w();

      if(broadcast_tf_) {

        AprilTagDescription description = des_pair.second;
        std::string child_frame_id = description.frame_name();
        if(child_frame_id.empty())
          child_frame_id = "tag_" + std::to_string(apriltag.id);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header = apriltags_msg.header;
        transformStamped.child_frame_id = child_frame_id;

        transformStamped.transform.translation.x = tvec.at<double>(0);
        transformStamped.transform.translation.y = tvec.at<double>(1);
        transformStamped.transform.translation.z = tvec.at<double>(2);

        transformStamped.transform.rotation.x = tf_quat.x();
        transformStamped.transform.rotation.y = tf_quat.y();
        transformStamped.transform.rotation.z = tf_quat.z();
        transformStamped.transform.rotation.w = tf_quat.w();

        tf2_br_->sendTransform(transformStamped);
      }

      apriltag_poses.apriltags.push_back(apriltag);
      apriltag_poses.posearray.poses.push_back(pose_msg);
    }
  }

  // The poses are apriltags expressed in camera frame
  apriltag_poses.header = apriltags_msg.header;
  apriltag_poses.posearray.header = apriltag_poses.header;
  if(apriltag_poses.apriltags.size()>0)
    pub_poses_->publish(apriltag_poses);
}

void ApriltagPoseEstimator::InitApriltagMap() {

  /*
  // Construct a map of strings
  std::map<std::string,double> map_tags;
  pnh_.getParam("tag_descriptions", map_tags);

  //Return if no tags are specified on parameter server.
  if(map_tags.empty())
    return;
  */
  std::vector<long int> ids;
  std::vector<double> tag_sizes;
  std::vector<std::string> frame_ids;
  ids = this->declare_parameter("tag_descriptions.ids",std::vector<int>());
  tag_sizes = this->declare_parameter("tag_descriptions.sizes",std::vector<double>());
  frame_ids = this->declare_parameter("tag_descriptions.frame_ids",std::vector<std::string>());

  std::map<int, AprilTagDescription> descriptions;
  if( ids.empty() ){
    RCLCPP_WARN(this->get_logger(), "No april tags specified");
    return;
  }
  else if( ids.size() != tag_sizes.size() || tag_sizes.size()!=frame_ids.size()){
    RCLCPP_WARN(this->get_logger(), "Mismatched item count in tag_descriptions");
    return;
  }
  else{
    try{
      descriptions = parse_tag_descriptions(ids, tag_sizes, frame_ids);
    } catch(XmlRpc::XmlRpcException e){
      RCLCPP_ERROR_STREAM(this->get_logger(),"Error loading tag descriptions: "<<e.getMessage());
    }
  }

  std::map<int,AprilTagDescription>::iterator description_itr;
  for(description_itr = descriptions.begin(); description_itr != descriptions.end(); description_itr++) {

    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    am::Apriltag tag;
    tag.family = "mit"; //TODO make these params? Or get from detections?
    tag.border = 1;
    tag.bits = 6;

    tag.id = description_itr->first;
    tag.center.x = 0.0;
    tag.center.y = 0.0;
    tag.center.z = 0.0;

    double s = tag_size/2.0;
    geometry_msgs::msg::Point pt;
    pt.x = -s; pt.y = -s; pt.z = 0.0; tag.corners[0] = pt;
    pt.x =  s; pt.y = -s; pt.z = 0.0; tag.corners[1] = pt;
    pt.x =  s; pt.y =  s; pt.z = 0.0; tag.corners[2] = pt;
    pt.x = -s; pt.y =  s; pt.z = 0.0; tag.corners[3] = pt;

    //TODO Only save the frame_id from description as string?
    auto des = std::make_pair(tag, description);
    map_.insert(std::pair<int, std::pair<am::Apriltag, AprilTagDescription> >(tag.id, des) );

    RCLCPP_INFO_STREAM(this->get_logger(), "tag is: " << am::to_yaml(tag) );
  }

  RCLCPP_INFO(this->get_logger(),"%s: apritlag map initialized", this->get_namespace());
}

std::map<int, AprilTagDescription>
 ApriltagPoseEstimator::parse_tag_descriptions(
      const std::vector<long int> &ids,
      const std::vector<double> &tag_sizes,
      const std::vector<std::string> &frame_ids ){
  std::map<int, AprilTagDescription> descriptions;
  for (int32_t i = 0; i < ids.size(); ++i) {
    int id = ids[i];
    double size = tag_sizes[i];
    std::string frame_name = frame_ids[i];
    if( frame_name.empty() ){
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    RCLCPP_INFO_STREAM(this->get_logger(),"Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}

void ApriltagPoseEstimator::CinfoCb(
    const sensor_msgs::msg::CameraInfo &cinfo_msg) {
  if (!cam_model_.initialized()) {
    cam_model_.fromCameraInfo(cinfo_msg);
    RCLCPP_INFO(this->get_logger(),"%s: camera model initialized", this->get_namespace());
    sub_cinfo_.reset(); // Stop subscription, ROS2 doesn't have shutdown() yet
  }
}

}  // namespace arpiltag_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<apriltag_ros::ApriltagPoseEstimator>(
      rclcpp::NodeOptions());
  RCLCPP_INFO(node->get_logger(), "apriltag detector started up!");
  // actually run the node
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return (0);
}
