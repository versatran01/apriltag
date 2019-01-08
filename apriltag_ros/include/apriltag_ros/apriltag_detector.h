#pragma once

#include <map>
#include <memory>

#include <apriltag_msgs/Apriltag.h>
#include <apriltag/apriltag.hpp>
#include <opencv2/core/core.hpp>

namespace apriltag_ros {

/// For now only support old tag family
enum class ApriltagFamily { tag36h11, tag25h9, tag16h5 };
using apriltag_msgs::Apriltag;

class ApriltagDetector {
 public:
  using SPtr = std::shared_ptr<ApriltagDetector>;
  using UPtr = std::unique_ptr<ApriltagDetector>;

  ApriltagDetector() = default;
  explicit ApriltagDetector(const ApriltagFamily &family) { AddFamily(family); }
  //  explicit ApriltagDetector(const std::vector<ApriltagFamily> &families);

  int nthreads() const { return td_->nthreads; }
  void set_nthreads(int nthreads) { td_->nthreads = nthreads; }

  int decimate() const { return td_->quad_decimate; }
  void set_decimate(int decimate) { td_->quad_decimate = decimate; }

  double sigma() const { return td_->quad_sigma; }
  void set_sigma(double sigma) { td_->quad_sigma = sigma; }

  /**
   * @brief Detect detects apriltags in given image
   * @param gray A grayscale image
   * @note corner starts from lower-left and goes counter-clockwise, and detect
   * does not need knowledge of the camera intrinsics
   */
  std::vector<Apriltag> Detect(const cv::Mat &gray, int max_hamming) const;

  size_t NumFamilies() const { return tfs_.size(); }
  bool AddFamily(const ApriltagFamily &family);
  bool RemoveFamily(const ApriltagFamily &family);
  void ClearFamily();

 private:
  apriltag::FamilyUPtr MakeFamily(const ApriltagFamily &family) const;

  apriltag::DetectorUPtr td_{apriltag_detector_create()};
  /// A set of all the families
  std::map<ApriltagFamily, apriltag::FamilyUPtr> tfs_;
};

/// Draw a single apriltag on image
void DrawApriltag(cv::Mat &image, const Apriltag &apriltag, int thickness = 2,
                  bool draw_corners = true, bool draw_id = true);

/// Draw a vector of apriltags on image
void DrawApriltags(cv::Mat &image, const std::vector<Apriltag> &apriltags,
                   int thickness = 2, bool draw_corners = true,
                   bool draw_id = true);

}  // namespace apriltag_ros
