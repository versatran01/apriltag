#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_H_

#include <cassert>
#include <memory>

#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_msgs/Apriltag.h>
#include <apriltag_umich/apriltag_umich.h>
#include <opencv2/core/core.hpp>

namespace apriltag_ros {

using ApriltagVec = std::vector<apriltag_msgs::Apriltag>;

enum class DetectorType { Mit, Umich };
enum class TagFamily { tf36h11, tf25h9, tf16h5 };

/**
 * @brief The ApriltagDetector class
 */
class ApriltagDetector {
public:
  using Ptr = boost::shared_ptr<ApriltagDetector>;

  ApriltagDetector(const DetectorType &detector_type,
                   const TagFamily &tag_family);
  virtual ~ApriltagDetector() = default;

  void set_black_border(int black_border);
  int black_border() const;

  void set_decimate(int decimate);
  int decimate() const;

  void set_nthreads(int nthreads);
  int nthreads() const;

  void print_profiling_info() const;

  int payload() const;
  const std::string &tag_family() const;

  /**
   * @brief Detect detects apriltags in given image
   * @param image A grayscale image
   * @note corner starts from lower-left and goes counter-clockwise, and detect
   * does not need knowledge of the camera intrinsics
   */
  ApriltagVec Detect(const cv::Mat &image);

  /**
   * @brief Create creates an instance of ApriltagDetector
   * @param type mit or umich
   * @param tag_family 36h11, 25h9, 16h5
   */
  static Ptr Create(const DetectorType &detector_type,
                    const TagFamily &tag_family);

protected:
  virtual ApriltagVec DetectImpl(const cv::Mat &image) = 0;
  virtual void SetBlackBorder(int black_border) = 0;
  virtual void SetDecimate(int decimate) = 0;
  virtual void SetNThreads(int nthreads) = 0;
  virtual void PrintProfilingInfo() const = 0;

  int decimate_{1};
  int nthreads_{1};
  int black_border_{1};
  int payload_{6};
  DetectorType detector_type_;
  TagFamily tag_family_;
  std::string tag_family_str_;
};

using ApriltagDetectorPtr = ApriltagDetector::Ptr;

/**
 * @brief The ApriltagDetectorMit class
 */
class ApriltagDetectorMit : public ApriltagDetector {
public:
  explicit ApriltagDetectorMit(const TagFamily &tag_family);

  ApriltagVec DetectImpl(const cv::Mat &image) override;

  void SetBlackBorder(int black_border) override;
  void SetDecimate(int decimate) override;
  void SetNThreads(int nthreads) override;
  void PrintProfilingInfo() const override;

private:
  apriltag_mit::TagDetectorPtr tag_detector_;
};

/**
 * @brief The ApriltagDetectorUmich class
 */
class ApriltagDetectorUmich : public ApriltagDetector {
public:
  explicit ApriltagDetectorUmich(const TagFamily &tag_family);

  ApriltagVec DetectImpl(const cv::Mat &image) override;

  void SetBlackBorder(int black_border) override;
  void SetDecimate(int decimate) override;
  void SetNThreads(int nthreads) override;
  void PrintProfilingInfo() const override;

private:
  apriltag_umich::TagFamilyPtr tag_family_;
  apriltag_umich::TagDetectorPtr tag_detector_;
};

/// Draw a single apriltag on image
void DrawApriltag(cv::Mat &image, const apriltag_msgs::Apriltag &apriltag,
                  int thickness = 2, bool draw_corners = true);

/// Draw a vector of apriltags on image
void DrawApriltags(cv::Mat &image, const ApriltagVec &apriltags);

/// Convert tag family to tag bits, eg. tf36h11 -> 6
int TagFamilyToPayload(const TagFamily &tag_family);

/// Convert detector type to string
std::string DetectorTypeToString(const DetectorType &detector_type);

/// Disable
// void RefineApriltags(const cv::Mat &image, ApriltagVec &apriltags,
//                     int win_size = 5);

} // namespace apriltag_ros

#endif // APRILTAG_ROS_APRILTAG_DETECTOR_H_
