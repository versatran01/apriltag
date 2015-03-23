#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_H_

#include <memory>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_msgs/Apriltag.h>

namespace apriltag_ros {

using ApriltagVec = std::vector<apriltag_msgs::Apriltag>;
/**
 * @brief The ApriltagDetector class
 */
class ApriltagDetector {
 public:
  using Ptr = std::unique_ptr<ApriltagDetector>;

  ApriltagDetector() = default;
  explicit ApriltagDetector(const std::string& type);
  virtual ~ApriltagDetector() = default;

  void set_decimate(double decimate) {
    assert(decimate >= 1);  // Decimation less than 1 doesn't make sense
    decimate_ = decimate;
  }

  void set_refine(bool refine) { refine_ = refine; }

  /**
   * @brief Detect detects apriltags in given image
   * @param image A grayscale image
   * @return detected tags without poses
   */
  ApriltagVec Detect(const cv::Mat& image);

  /**
   * @brief Draw draws detected tags on given image
   * @param image A grayscale/color image
   */
  //  virtual void Draw(cv::Mat& image) = 0;

  const std::string& type() const { return type_; }

  static Ptr create(const std::string& type);

 protected:
  virtual ApriltagVec DetectImpl(const cv::Mat& image) = 0;

  double decimate_{1.0};
  bool refine_{true};
  std::string type_;
};

using ApriltagDetectorPtr = ApriltagDetector::Ptr;

/**
 * @brief The ApriltagDetectorMit class
 */
class ApriltagDetectorMit : public ApriltagDetector {
 public:
  explicit ApriltagDetectorMit(const apriltag_mit::TagCodes& tag_codes);

  virtual ApriltagVec DetectImpl(const cv::Mat& image) override;

 private:
  void RefineDetections(const cv::Mat& image);

  apriltag_mit::TagDetector tag_detector_;
  std::vector<apriltag_mit::TagDetection> tag_detections_;
};

/**
 * @brief The ApriltagDetectorUmich class
 */
class ApriltagDetectorUmich : public ApriltagDetector {
 public:
};

}  // apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_H_
