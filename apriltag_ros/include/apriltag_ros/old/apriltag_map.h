#ifndef APRILTAG_ROS_APRILTAG_MAP_H_
#define APRILTAG_ROS_APRILTAG_MAP_H_

#include "apriltag_ros/apriltag_detection.h"
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <map>
#include <tuple>

namespace apriltag_ros {

/**
 * @brief The ApriltagMap class
 */
class ApriltagMap {
 public:
  /**
   * @brief The Tag3D class
   */
  class Tag3D {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Matrix34d = Eigen::Matrix<double, 3, 4>;
    Tag3D() = default;
    Tag3D(int id) : id_(id) {}
    Tag3D(int id, const Eigen::Quaterniond& q, const Eigen::Vector3d& p)
        : id_(id), q_(q), p_(p) {}

    int id() const { return id_; }
    void set_id(int id) { id_ = id; }

    const Eigen::Quaterniond& q() const { return q_; }
    void set_q(const Eigen::Quaterniond& q) { q_ = q; }

    const Eigen::Vector3d& p() const { return p_; }
    void set_p(const Eigen::Vector3d& p) { p_ = p; }

    const Matrix34d& corners() const { return corners_; }
    Matrix34d& corners() { return corners_; }

    /**
     * @brief update Update corners position
     */
    void update(double tag_size);

   private:
    int id_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d p_;
    Matrix34d corners_;
  };

  using QPB = std::tuple<Eigen::Quaterniond, Eigen::Vector3d, bool>;
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
  const std::map<int, Tag3D>& tag_map() const { return tag_map_; }

  /**
   * @brief addTag add a tag to the map based on the id
   */
  void addTag(const Tag3D& tag);

  QPB estimatePose(const std::vector<ApriltagDetection>& detections);

 private:
  std::string tag_family_;
  double tag_size_;
  std::map<int, Tag3D> tag_map_;
};

/**
 * @brief loadApriltagMapYaml
 * @param filename
 * @return ApriltagMap
 */
ApriltagMap loadApriltagMapYaml(const std::string& filename);

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_MAP_H_
