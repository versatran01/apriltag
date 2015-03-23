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

  ApriltagDetector(const std::string& type, const std::string& tag_family);
  virtual ~ApriltagDetector() = default;

  void set_decimate(double decimate) {
    decimate_ = (decimate >= 1) ? decimate : 1;
  }
  const double decimate() const { return decimate_; }

  void set_refine(bool refine) { refine_ = refine; }
  const bool refine() { return refine_; }

  void set_tag_size(double tag_size) { tag_size_ = tag_size; }
  const double tag_size() const { return tag_size_; }

  const std::string& tag_family() const { return tag_family_; }
  const std::string& type() const { return type_; }

  /**
   * @brief Detect detects apriltags in given image
   * @param image A grayscale image
   * @return detected tags without poses
   * @note corner starts from lower-left and goes counter-clockwise
   */
  ApriltagVec Detect(const cv::Mat& image);

  /**
   * @brief Draw draws detected tags on given image
   * @param image A grayscale/color image
   */
  void Draw(cv::Mat& image) const;

  static Ptr create(const std::string& type, const std::string& tag_family);

 protected:
  virtual ApriltagVec DetectImpl(const cv::Mat& image) = 0;
  virtual void DrawImpl(cv::Mat& image) const = 0;

  double decimate_{1.0};
  bool refine_{true};
  double tag_size_{0};
  std::string type_;
  std::string tag_family_;
};

using ApriltagDetectorPtr = ApriltagDetector::Ptr;

/**
 * @brief The ApriltagDetectorMit class
 */
class ApriltagDetectorMit : public ApriltagDetector {
 public:
  explicit ApriltagDetectorMit(const std::string& tag_family);

  virtual ApriltagVec DetectImpl(const cv::Mat& image) override;
  virtual void DrawImpl(cv::Mat& image) const override;

 private:
  /**
   * @brief RefineDetections Refine corners using opencv cornerSubPix
   */
  void RefineDetections(const cv::Mat& image);

  /**
   * @brief TagDetectionsToApriltagMsg Convert TagDetections to Apriltag
   */
  ApriltagVec TagDetectionsToApriltagMsg() const;

  apriltag_mit::TagDetectorPtr tag_detector_;
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
