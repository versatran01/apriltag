#ifndef APRILTAG_ROS_APRILTAG_DETECTION_H_
#define APRILTAG_ROS_APRILTAG_DETECTION_H_

#include <limits>

#include <apriltag_msgs/Apriltag.h>
#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_umich/apriltag_umich.h>

namespace Eigen {
#define EIGEN_MAKE_PARAMETERIZED_TYPEDEFS(Size, SizeSuffix) \
  template <typename T>                                    \
  using Matrix##SizeSuffix = Matrix<T, Size, Size>;        \
  template <typename T>                                    \
  using Vector##SizeSuffix = Matrix<T, Size, 1>;           \
  template <typename T>                                    \
  using RowVector##SizeSuffix = Matrix<T, 1, Size>;

#define EIGEN_MAKE_PARAMETERIZED_FIXED_TYPEDEFS(Size) \
  template <typename T>                               \
  using Matrix##Size##X = Matrix<T, Size, Dynamic>;   \
  template <typename T>                               \
  using Matrix##X##Size = Matrix<T, Dynamic, Size>;


#define EIGEN_MAKE_PARAMETERIZED_TYPEDEFS_ALL_SIZES() \
EIGEN_MAKE_PARAMETERIZED_TYPEDEFS(2, 2) \
EIGEN_MAKE_PARAMETERIZED_TYPEDEFS(3, 3) \
EIGEN_MAKE_PARAMETERIZED_TYPEDEFS(4, 4) \
EIGEN_MAKE_PARAMETERIZED_TYPEDEFS(Dynamic, X) \
EIGEN_MAKE_PARAMETERIZED_FIXED_TYPEDEFS(2) \
EIGEN_MAKE_PARAMETERIZED_FIXED_TYPEDEFS(3) \
EIGEN_MAKE_PARAMETERIZED_FIXED_TYPEDEFS(4)

EIGEN_MAKE_PARAMETERIZED_TYPEDEFS_ALL_SIZES()
}

namespace apriltag_ros {
/**
 * @brief The ApriltagDetection class
 */
class ApriltagDetection {
 public:
  using RtPair = std::pair<Eigen::Matrix3d, Eigen::Matrix3d>;

  ApriltagDetection() = default;
  explicit ApriltagDetection(const apriltag_mit::TagDetection& td);
  explicit ApriltagDetection(const apriltag_detection_t* td);
  explicit operator apriltag_msgs::Apriltag() const;

  void draw(cv::Mat& image, int thickness = 1) const;
  void drawLine(cv::Mat& image, int b, int e, const cv::Scalar& color,
                int thickness = 1) const;

  void estimate(const cv::Matx33d& K, double tag_size);

  int id;
  int hamming;
  double c[2];
  double p[4][2];
  double size;
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
};

template <typename T>
Eigen::AngleAxis<T> RotationVectorToAngleAxis(const Eigen::Vector3<T>& r) {
  static_assert(std::is_floating_point<T>::value, "T must be floating point");
  const auto angle = r.norm();
  Eigen::Matrix<T, 3, 1> axis(0, 0, 0);
  if (angle > std::numeric_limits<T>::epsilon() * 10) axis = r / angle;
  return Eigen::AngleAxis<T>(angle, axis);
}

template <typename T>
Eigen::Quaternion<T> RotationVectorToQuaternion(const Eigen::Vector3<T>& r) {
  static_assert(std::is_floating_point<T>::value, "T must be floating point");
  Eigen::Quaternion<T> q(RotationVectorToAngleAxis(r));
  return q;
}

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTION_H_
