#include "apriltag_ros/apriltag_map.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sv_base/math.h>

namespace apriltag_ros {
Eigen::Quaterniond RodriguesToQuat(const cv::Mat& rvec) {
  Eigen::Vector3d r(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
  // Copied from kr_math pose
  const double rn = r.norm();
  Eigen::Vector3d rnorm(0.0, 0.0, 0.0);
  if (rn > std::numeric_limits<double>::epsilon() * 10) rnorm = r / rn;
  return Eigen::Quaterniond(Eigen::AngleAxis<double>(rn, rnorm));
}

/// ===========
/// ApriltagMap
/// ===========
void ApriltagMap::addTag(const Tag3D& tag) {
  if (tag_map_.find(tag.id()) == tag_map_.end()) {
    // Not found add to map
    tag_map_[tag.id()] = tag;
  } else {
    // Already there, throw
    throw std::runtime_error("id already exists" + std::to_string(tag.id()));
  }
}

ApriltagMap::QPB ApriltagMap::estimatePose(
    const std::vector<ApriltagDetection>& detections) {
  if (detections.empty())
    return std::make_tuple(Eigen::Quaterniond(), Eigen::Vector3d(), false);

  std::vector<cv::Point2d> p_cam;  // points in camera coordinates
  std::vector<cv::Point3d> p_tag;  // points in world coordinates
  p_cam.reserve(detections.size() * 4);
  p_tag.reserve(detections.size() * 4);

  for (const ApriltagDetection& td : detections) {
    const Tag3D& tag_3d = tag_map_[td.id];
    assert(td.id == tag_3d.id());
    const auto& c = tag_3d.corners();
    for (int i = 0; i < 4; ++i) {
      p_cam.push_back({td.n[i][0], td.n[i][1]});
      p_tag.push_back({c(0, i), c(1, i), c(2, i)});
    }
  }

  cv::Mat rvec, tvec;
  const cv::Mat E = cv::Mat::eye(3, 3, CV_64FC1);
  cv::solvePnP(p_tag, p_cam, E, cv::noArray(), rvec, tvec);
  // The estimated r and t brings points from tag frame to camera frame, but we
  // are interested in the inverse transformation
  // So the r here is c_r_w, and t here is c_t_w
  Eigen::Vector3d c_r_w, c_t_w;
  cv::cv2eigen(rvec, c_r_w);
  cv::cv2eigen(tvec, c_t_w);
  Eigen::Quaterniond w_q_c(
      sv::base::RotationVectorToQuaternion(c_r_w).conjugate());
  // Cannot use const auto here, does not initialize
  Eigen::Vector3d w_t_c(-w_q_c.toRotationMatrix() * c_t_w);
  return std::make_tuple(w_q_c, w_t_c, true);
}

void ApriltagMap::Tag3D::update(double tag_size) {
  assert(tag_size > 0);
  double s = tag_size / 2;
  const auto w_R_t = q_.toRotationMatrix();  // Tag to World rotation
  Matrix34d c_t;
  c_t.col(0) << -s, -s, 0;
  c_t.col(1) << s, -s, 0;
  c_t.col(2) << s, s, 0;
  c_t.col(3) << -s, s, 0;

  for (int i = 0; i < 4; ++i) {
    corners_.col(i) = w_R_t * c_t.col(i) + p_;
  }
}

ApriltagMap loadApriltagMapYaml(const std::string& filename) {
  const auto file = YAML::LoadFile(filename);
  const auto tag_family = file["tag_family"].as<std::string>();
  const auto tag_size = file["tag_size"].as<double>();
  ApriltagMap apriltag_map(tag_family, tag_size);

  const YAML::Node& tags = file["tags"];
  for (size_t i = 0; i < tags.size(); ++i) {
    const YAML::Node& tag = tags[i];
    const auto id = tag["id"].as<int>();
    const long code = tag["code"].as<long>();
    const YAML::Node& orientation = tag["orientation"];
    const YAML::Node& position = tag["position"];

    ApriltagMap::Tag3D tag_3d(id);
    {
      const auto w = orientation["w"].as<double>();
      const auto x = orientation["x"].as<double>();
      const auto y = orientation["y"].as<double>();
      const auto z = orientation["z"].as<double>();
      tag_3d.set_q({w, x, y, z});
    }

    {
      const auto x = position["x"].as<double>();
      const auto y = position["y"].as<double>();
      const auto z = position["z"].as<double>();
      tag_3d.set_p({x, y, z});
    }

    tag_3d.update(tag_size);
    apriltag_map.addTag(tag_3d);
    std::cout << "add tag: " << id << "/" << code << std::endl;
  }
  return apriltag_map;
}

}  // namespace apriltag_ros
