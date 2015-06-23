#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_H_

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

  ApriltagDetector(const std::string& detector_type,
                   const std::string& tag_family);
  virtual ~ApriltagDetector() = default;

  void set_black_border(int black_border) {
    black_border_ = black_border;
    setBlackBorder(black_border);
  }
  int black_border() const { return black_border_; }

  void set_tag_bits(int tag_bits) { tag_bits_ = tag_bits; }
  int tag_bits() const { return tag_bits_; }

  void set_decimate(int decimate) {
    decimate_ = (decimate >= 1) ? decimate : 1;
  }
  const int decimate() const { return decimate_; }

  void set_refine(bool refine) { refine_ = refine; }
  const bool refine() { return refine_; }

  const std::string& tag_family() const { return tag_family_; }
  const std::string& detector_type() const { return detector_type_; }

  /**
   * @brief Detect detects apriltags in given image
   * @param image A grayscale image
   * @note corner starts from lower-left and goes counter-clockwise, and detect
   * does not need knowledge of the camera intrinsics
   */
  void detect(const cv::Mat& image);

  /**
   * @brief Draw draws detected tags on given image
   * @param image A grayscale/color image
   */
  void draw(cv::Mat& image) const;

  /**
   * @brief empty If the current detection is empty
   * @return
   */
  bool empty() const { return apriltags_.empty(); }

  ApriltagVec apriltags() const { return apriltags_; }

  /**
   * @brief Create creates an instance of ApriltagDetector
   * @param type mit or umich
   * @param tag_family 36h11, 25h9, 16h5
   */
  static Ptr create(const std::string& detector_type,
                    const std::string& tag_family);

 protected:
  virtual void detectImpl(const cv::Mat& image) = 0;
  virtual void setBlackBorder(int black_border) = 0;

  int decimate_{1};
  bool refine_{false};
  int black_border_{1};
  int tag_bits_{6};  // default to t36h11
  std::string detector_type_;
  std::string tag_family_;
  ApriltagVec apriltags_;
};

using ApriltagDetectorPtr = ApriltagDetector::Ptr;

/**
 * @brief The ApriltagDetectorMit class
 */
class ApriltagDetectorMit : public ApriltagDetector {
 public:
  explicit ApriltagDetectorMit(const std::string& tag_family);

  void detectImpl(const cv::Mat& image) override;

  void setBlackBorder(int black_border) override {
    tag_detector_->setBlackBorder(black_border);
  }

 private:
  apriltag_mit::TagDetectorPtr tag_detector_;
};

/**
 * @brief The ApriltagDetectorUmich class
 */
class ApriltagDetectorUmich : public ApriltagDetector {
 public:
  explicit ApriltagDetectorUmich(const std::string& tag_family);

  void detectImpl(const cv::Mat& image) override;

  void setBlackBorder(int black_border) override {
    tag_family_->black_border = black_border;
  }

 private:
  apriltag_umich::TagFamilyPtr tag_family_;
  apriltag_umich::TagDetectorPtr tag_detector_;
};

/**
 * @brief drawApriltag
 * @param image
 * @param apriltag
 */
void drawApriltag(cv::Mat& image, const apriltag_msgs::Apriltag& apriltag,
                  int thickness = 1);

}  // apriltag_ros

#endif  // APRILTAG_ROS_APRILTAG_DETECTOR_H_
