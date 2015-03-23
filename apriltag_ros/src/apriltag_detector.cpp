#include "apriltag_ros/apriltag_detector.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {
/// ApriltagDetector
ApriltagDetector::ApriltagDetector(const string& type) : type_(type) {}

static ApriltagDetector::ApriltagDetectorPtr create(const std::string& type) {}

/// ApriltagDetectorMit
ApriltagDetectorMit::ApriltagDetectorMit(const TagCodes& tag_codes)
    : ApriltagDetector("mit"), tag_detector_(tag_codes) {}

ApriltagVec ApriltagDetectorMit::Detect(const cv::Mat& image) {
  ApriltagVec apriltag_vec;
  if (image.empty()) return apriltag_vec;

  // Decimate image
  cv::Mat gray;
  if (decimate_ != 1.0) {
    cv::resize(image, gray, cv::Size(0, 0), 1 / decimate_, 1 / decimate_);
  } else {
    gray = image;
  }

  // Detection
  tag_detections_ = tag_detector_.extractTags(gray);

  // Handle decimation
  if (decimate_ != 1.0) {
    for (TagDetection& td : tag_detections_) {
      td.Scale(decimate_);
    }
  }

  // Refine corners
  if (refine_) {
    std::vector<cv::Point2f> corners;
    for (const TagDetection& td : tag_detections_) {
      for (const auto& p : td.p) {
        corners.push_back(cv::Point2f(p.first, p.second));
      }
    }
    const auto criteria =
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    decltype(corners.size()) i = 0;
    for (TagDetection& td : tag_detections_) {
      decltype(td.p.first) sum_x{0.0}, sum_y{0.0};
      for (auto& p : td.p) {
        p.first = corners[i].x;
        p.second = corners[i].y;
        sum_x += p.first;
        sum_y += p.second;
        ++i;
      }
      td.cxy.first = sum_x / 4;
      td.cxy.second = sum_y / 4;
    }
  }

  return apriltag_vec;
}

/// ApriltagDetectorUmich
}  // namespace apriltag_ros
