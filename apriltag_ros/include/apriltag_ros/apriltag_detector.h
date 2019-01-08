#pragma once

#include <map>
#include <memory>

#include <apriltag/apriltag.hpp>
#include <opencv2/core/core.hpp>

namespace apriltag_ros {

/// For now only support old tag family
enum class ApriltagFamily { tf36h11, tf25h9, tf16h5 };

class ApriltagDetector {
 public:
  using SPtr = std::shared_ptr<ApriltagDetector>;
  using UPtr = std::unique_ptr<ApriltagDetector>;

  ApriltagDetector() = default;
  explicit ApriltagDetector(const ApriltagFamily &family) { AddFamily(family); }
  //  explicit ApriltagDetector(const std::vector<ApriltagFamily> &families);

  void set_nthreads(int nthreads) { td_->nthreads = nthreads; }
  void set_decimate(int decimate) { td_->quad_decimate = decimate; }
  void set_sigma(double sigma) { td_->quad_sigma = sigma; }

  /**
   * @brief Detect detects apriltags in given image
   * @param gray A grayscale image
   * @note corner starts from lower-left and goes counter-clockwise, and detect
   * does not need knowledge of the camera intrinsics
   */
  void Detect(const cv::Mat &gray);

  /**
   * @brief Create creates an instance of ApriltagDetector
   * @param type mit or umich
   * @param tag_family 36h11, 25h9, 16h5
   */
  //  static SPtr Create(const TagFamily &tag_family);
  bool AddFamily(const ApriltagFamily &family);
  bool RemoveFamily(const ApriltagFamily &family);

 private:
  apriltag::FamilyUPtr MakeFamily(const ApriltagFamily &family) const;

  apriltag::DetectorUPtr td_{apriltag_detector_create()};
  /// A set of all the families
  std::map<ApriltagFamily, apriltag::FamilyUPtr> tfs_;
};

/// Draw a single apriltag on image
// void DrawApriltag(cv::Mat &image, const apriltag_msgs::Apriltag &apriltag,
//                  int thickness = 2, bool draw_corners = true);

/// Draw a vector of apriltags on image
// void DrawApriltags(cv::Mat &image, const ApriltagVec &apriltags);

/// Convert tag family to tag bits, eg. tf36h11 -> 6
// int TagFamilyToPayload(const TagFamily &tag_family);

/// Convert detector type to string
// std::string DetectorTypeToString(const DetectorType &detector_type);

}  // namespace apriltag_ros
