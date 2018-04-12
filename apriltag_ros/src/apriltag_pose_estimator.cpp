#include "apriltag_ros/apriltag_pose_estimator.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/thread/lock_guard.hpp>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <XmlRpcException.h>


namespace apriltag_ros {

namespace am = apriltag_msgs;
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

ApriltagPoseEstimator::ApriltagPoseEstimator(const ros::NodeHandle &pnh)
    : pnh_(pnh) {

  pnh_.param("broadcast_tf", broadcast_tf_, false);
  bool auto_disconnect = true;
  pnh_.param("auto_disconnect", auto_disconnect, true);

  sub_cinfo_ = pnh_.subscribe("camera_info", 1, &ApriltagPoseEstimator::CinfoCb, this);

  if(auto_disconnect){
    auto connect_cb = boost::bind(&ApriltagPoseEstimator::ConnectCb, this);
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_poses_ = pnh_.advertise<apriltag_msgs::ApriltagPoseStamped>("apriltag_poses", 1, connect_cb, connect_cb);
  }
  else{
    pub_poses_ = pnh_.advertise<apriltag_msgs::ApriltagPoseStamped>("apriltag_poses", 1);
    ROS_WARN("%s: Subscribing", pnh_.getNamespace().c_str());
    sub_apriltags_ = pnh_.subscribe("apriltags", 1,
                                    &ApriltagPoseEstimator::ApriltagsCb, this);
  }

  InitApriltagMap();
}

void ApriltagPoseEstimator::ConnectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_poses_.getNumSubscribers() == 0) {
    ROS_WARN("%s: No subscribers, shutting down", pnh_.getNamespace().c_str());
    sub_apriltags_.shutdown();
  } else if (!sub_apriltags_) {
    ROS_WARN("%s: Resubscribing", pnh_.getNamespace().c_str());
    sub_apriltags_ = pnh_.subscribe("apriltags", 1,
                                    &ApriltagPoseEstimator::ApriltagsCb, this);
  }
}

void ApriltagPoseEstimator::ApriltagsCb(const am::ApriltagArrayStampedConstPtr &apriltags_msg) {
  if (!cam_model_.initialized() || map_.empty()) return;

  am::ApriltagPoseStamped apriltag_poses;

  for (const am::Apriltag &apriltag : apriltags_msg->apriltags) {
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

      // Solve for pose
      // The estiamted r and t brings points from tag frame to camera frame
      // r = c_r_w, t = c_t_w
      cv::Mat rvec, tvec;
      auto good = cv::solvePnP(obj_pts, img_pts, cam_model_.fullIntrinsicMatrix(),
                               cam_model_.distortionCoeffs(), rvec, tvec);
      if (!good) {
        ROS_WARN("%s: Pose solver failed.", pnh_.getNamespace().c_str());
        continue;
      }

      cv::Matx33d r;
      cv::Rodrigues(rvec, r);

      tf2::Matrix3x3 wRo(r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2));
      tf2::Quaternion tf_quat;
      wRo.getRotation(tf_quat);

      geometry_msgs::Pose pose_msg;
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

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header = apriltags_msg->header;
        transformStamped.child_frame_id = child_frame_id;

        transformStamped.transform.translation.x = tvec.at<double>(0);
        transformStamped.transform.translation.y = tvec.at<double>(1);
        transformStamped.transform.translation.z = tvec.at<double>(2);

        transformStamped.transform.rotation.x = tf_quat.x();
        transformStamped.transform.rotation.y = tf_quat.y();
        transformStamped.transform.rotation.z = tf_quat.z();
        transformStamped.transform.rotation.w = tf_quat.w();

        tf2_br_.sendTransform(transformStamped);
      }

      apriltag_poses.apriltags.push_back(apriltag);
      apriltag_poses.posearray.poses.push_back(pose_msg);
    }
  }

  // The poses are apriltags expressed in camera frame
  apriltag_poses.header = apriltags_msg->header;
  apriltag_poses.posearray.header = apriltag_poses.header;
  if(apriltag_poses.apriltags.size()>0)
    pub_poses_.publish(apriltag_poses);
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

  std::map<int, AprilTagDescription> descriptions;

  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh_.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
    return;
  }
  else{
    try{
      descriptions = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
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
    geometry_msgs::Point pt;
    pt.x = -s; pt.y = -s; pt.z = 0.0; tag.corners[0] = pt;
    pt.x =  s; pt.y = -s; pt.z = 0.0; tag.corners[1] = pt;
    pt.x =  s; pt.y =  s; pt.z = 0.0; tag.corners[2] = pt;
    pt.x = -s; pt.y =  s; pt.z = 0.0; tag.corners[3] = pt;

    //TODO Only save the frame_id from description as string?
    auto des = std::make_pair(tag, description);
    map_.insert(std::pair<int, std::pair<am::Apriltag, AprilTagDescription> >(tag.id, des) );

    ROS_INFO_STREAM("tag is: " << tag);
  }

  ROS_INFO("%s: apritlag map initialized", pnh_.getNamespace().c_str());
}

std::map<int, AprilTagDescription> ApriltagPoseEstimator::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}

void ApriltagPoseEstimator::CinfoCb(
    const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  if (!cam_model_.initialized()) {
    cam_model_.fromCameraInfo(cinfo_msg);
    ROS_INFO("%s: camera model initialized", pnh_.getNamespace().c_str());
    sub_cinfo_.shutdown();
  }
}

}  // namespace arpiltag_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_pose_estimator");
  ros::NodeHandle pnh("~");

  try {
    apriltag_ros::ApriltagPoseEstimator node(pnh);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
