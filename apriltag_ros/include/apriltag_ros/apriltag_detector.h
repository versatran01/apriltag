#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_H_

#include <memory>

#include <opencv2/core/core.hpp>

namespace apriltag_ros {

class ApriltagDetector {
 public:
  using ApriltagDetectorPtr = std::unique_ptr<ApriltagDetector>;

  ApriltagDetector();
  virtual ~ApriltagDetector();

  virtual void Detect(const cv::Mat& image);

  /**
   * @brief create
   * @param type
   * @return
   */
  static ApriltagDetectorPtr create(const std::string& type);

 protected:
 private:
};

class ApriltagDetectorMit : ApriltagDetector {};

class ApriltagDetectorUmich : ApriltagDetector {};

}  // apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_H_
