#include "apriltag_ros/apriltag_pose_estimator.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/thread/lock_guard.hpp>
#include <geometry_msgs/TransformStamped.h>

namespace apriltag_ros {

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
  sub_cinfo_ =
      pnh_.subscribe("camera_info", 1, &ApriltagPoseEstimator::CinfoCb, this);

  auto connect_cb = boost::bind(&ApriltagPoseEstimator::ConnectCb, this);
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_poses_ = pnh_.advertise<apriltag_msgs::ApriltagPoseStamped>(
      "apriltag_poses", 1, connect_cb, connect_cb);
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

void ApriltagPoseEstimator::ApriltagsCb(
    const am::ApriltagArrayStampedConstPtr &apriltags_msg) {
  if (!cam_model_.initialized() || map_.empty()) return;

  am::ApriltagPoseStamped apriltag_poses;

  // Prepare points for solve pnp
  std::vector<cv::Point2d> img_pts;
  std::vector<cv::Point3d> obj_pts;
  for (const am::Apriltag &apriltag : apriltags_msg->apriltags) {
    auto search = map_.find(apriltag.id);
    if (search != map_.end()) {
      // Add points to img_pts
      const am::Apriltag &map_tag = search->second;
      for (const auto &c : apriltag.corners) {
        img_pts.emplace_back(c.x, c.y);
      }
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

      const auto cQw = QuatFromRvec(rvec);

      geometry_msgs::Pose pose_msg;
      const auto *pt = tvec.ptr<double>();
      pose_msg.position.x = pt[0];
      pose_msg.position.y = pt[1];
      pose_msg.position.z = pt[2];

      const auto *pq = cQw.ptr<double>();
      pose_msg.orientation.w = pq[0];
      pose_msg.orientation.x = pq[1];
      pose_msg.orientation.y = pq[2];
      pose_msg.orientation.z = pq[3];

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

  // Construct a map of strings
  std::map<std::string,double> map_tags;
  pnh_.getParam("tag_descriptions", map_tags);

  //Return if no tags are specified on parameter server.
  if(map_tags.empty())
    return;

  std::map<std::string,double>::iterator tags_it;
  for(tags_it = map_tags.begin(); tags_it != map_tags.end(); tags_it++)
  {
    am::Apriltag tag;
    tag.family = "mit"; //TODO make these params? Or get from detections?
    tag.border = 1;
    tag.bits = 6;

    tag.id = std::atoi(tags_it->first.c_str());
    tag.center.x = 0.0;
    tag.center.y = 0.0;
    tag.center.z = 0.0;

    double tag_size = tags_it->second;
    double s = tag_size/2.0;
    geometry_msgs::Point pt;
    pt.x = -s; pt.y = -s; pt.z = 0.0; tag.corners[0] = pt;
    pt.x =  s; pt.y = -s; pt.z = 0.0; tag.corners[1] = pt;
    pt.x =  s; pt.y =  s; pt.z = 0.0; tag.corners[2] = pt;
    pt.x = -s; pt.y =  s; pt.z = 0.0; tag.corners[3] = pt;

    map_[tag.id] = tag;

    ROS_INFO_STREAM("tag is: " << tag);
  }

  ROS_INFO("%s: apritlag map initialized", pnh_.getNamespace().c_str());
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
