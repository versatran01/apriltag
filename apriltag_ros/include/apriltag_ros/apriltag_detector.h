#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_H_

#include "apriltag_ros/apriltag_detection.h"

#include <memory>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_umich/apriltag_umich.h>
#include <apriltag_msgs/Apriltag.h>

namespace apriltag_ros {

using ApriltagVec = std::vector<apriltag_msgs::Apriltag>;

/**
 * @brief The ApriltagDetector class
 */
class ApriltagDetector {
 public:
  using Ptr = boost::shared_ptr<ApriltagDetector>;

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
  const std::vector<ApriltagDetection>& tag_detections() const {
    return tag_detections_;
  }

  /**
   * @brief Detect detects apriltags in given image
   * @param image A grayscale image
   * @note corner starts from lower-left and goes counter-clockwise
   */
  void Detect(const cv::Mat& image);

  /**
   * @brief Estimate estimates poses in camera frame
   */
  // TODO: implement this
  void Estimate(const cv::Matx33d& K, const cv::Mat_<double>& D);

  /**
   * @brief Draw draws detected tags on given image
   * @param image A grayscale/color image
   */
  void Draw(cv::Mat& image) const;

  /**
   * @brief ToApriltagMsgs convert internal tag detection to ros message
   */
  ApriltagVec ToApriltagMsg() const;

  /**
   * @brief Create creates an instance of ApriltagDetector
   * @param type mit or umich
   * @param tag_family 36h11, 25h9, 16h5
   */
  static Ptr Create(const std::string& type, const std::string& tag_family);

 protected:
  virtual void DetectImpl(const cv::Mat& image) = 0;

  double decimate_{1.0};
  bool refine_{false};
  double tag_size_{0};
  std::string type_;
  std::string tag_family_;
  std::vector<ApriltagDetection> tag_detections_;
};

using ApriltagDetectorPtr = ApriltagDetector::Ptr;

/**
 * @brief The ApriltagDetectorMit class
 */
class ApriltagDetectorMit : public ApriltagDetector {
 public:
  explicit ApriltagDetectorMit(const std::string& tag_family);

  virtual void DetectImpl(const cv::Mat& image) override;

 private:
  /**
   * @brief RefineDetections Refine corners using opencv cornerSubPix
   */
  void RefineDetections(const cv::Mat& image);

  apriltag_mit::TagDetectorPtr tag_detector_;
};

/**
 * @brief The ApriltagDetectorUmich class
 */
class ApriltagDetectorUmich : public ApriltagDetector {
 public:
  explicit ApriltagDetectorUmich(const std::string& tag_family);

  virtual void DetectImpl(const cv::Mat& image) override;

 private:
  apriltag_umich::TagFamilyPtr tag_family_;
  apriltag_umich::TagDetectorPtr tag_detector_;
};

}  // apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_H_
