#ifndef APRILTAG_ROS_APRILTAG_MAP_H_
#define APRILTAG_ROS_APRILTAG_MAP_H_

#include "apriltag_ros/apriltag_detection.h"
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <map>

namespace apriltag_ros {

class ApriltagMap {
 public:
  class Tag3D {
   public:
    Tag3D() = default;
    Tag3D(int id) : id_(id) {}
    Tag3D(int id, const Eigen::Quaterniond& q, const Eigen::Vector3d& p)
        : id_(id), q_(q), p_(p) {}

    int id() const { return id_; }
    const Eigen::Quaterniond& q() const { return q_; }
    void set_q(const Eigen::Quaterniond& q) { q_ = q; }
    const Eigen::Vector3d& p() const { return p_; }
    void set_p(const Eigen::Vector3d& p) { p_ = p; }

   private:
    int id_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d p_;
  };

  ApriltagMap() = default;
  ApriltagMap(const std::string& tag_family, double tag_size)
      : tag_family_(tag_family), tag_size_(tag_size) {}

  void set_tag_family(const std::string& tag_family) {
    tag_family_ = tag_family;
  }
  const std::string& tag_family() const { return tag_family_; }

  void set_tag_size(double tag_size) { tag_size_ = tag_size; }
  double tag_size() const { return tag_size_; }

  size_t size() const { return tag_map_.size(); }
  bool empty() const { return tag_map_.empty(); }

  /**
   * @brief addTag add a tag to the map based on the id
   */
  void addTag(const Tag3D& tag);

  void estimatePose(const std::vector<ApriltagDetection>& detections);

 private:
  std::string tag_family_;
  double tag_size_;
  std::map<int, Tag3D> tag_map_;
};

ApriltagMap loadApriltagMapYaml(const std::string& filename);

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_MAP_H_
