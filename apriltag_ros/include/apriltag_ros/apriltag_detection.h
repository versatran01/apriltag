#ifndef APRILTAG_ROS_APRILTAG_DETECTION_H_
#define APRILTAG_ROS_APRILTAG_DETECTION_H_

#include <limits>

#include <apriltag_msgs/Apriltag.h>
#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_umich/apriltag_umich.h>

namespace apriltag_ros {
/**
 * @brief The ApriltagDetection class
 */
class ApriltagDetection {
 public:
  ApriltagDetection() = default;
  explicit ApriltagDetection(const apriltag_mit::TagDetection& td);
  explicit ApriltagDetection(const apriltag_detection_t* td);
  explicit operator apriltag_msgs::Apriltag() const;

  void draw(cv::Mat& image, int thickness = 1) const;
  void drawLine(cv::Mat& image, int b, int e, const cv::Scalar& color,
                int thickness = 1) const;

  void estimate(double tag_size, const cv::Matx33d& K,
                const cv::Mat_<double>& D = cv::Mat(1, 5, 0.0));

  int id;
  int hamming;
  double c[2];
  double p[4][2];
  double size;
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTION_H_
