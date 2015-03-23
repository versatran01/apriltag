#include "apriltag_ros/apriltag_detector.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

namespace mit = apriltag_mit;

/// ApriltagDetector
ApriltagDetector::ApriltagDetector(const std::string& type,
                                   const std::string& tag_family)
    : type_(type), tag_family_(tag_family) {}

ApriltagVec ApriltagDetector::Detect(const cv::Mat& image) {
  if (image.empty()) return {};
  return DetectImpl(image);
}

void ApriltagDetector::Draw(cv::Mat& image) const { DrawImpl(image); }

static ApriltagDetector::Ptr create(const std::string& type,
                                    const std::string& tag_family) {}

/// ApriltagDetectorMit
ApriltagDetectorMit::ApriltagDetectorMit(const string& tag_family)
    : ApriltagDetector("mit", tag_family) {
  if (tag_family == "36h11") {
    tag_detector_.reset(new mit::TagDetector(mit::tagCodes36h11));
  } else if (tag_family == "25h9") {
    tag_detector_.reset(new mit::TagDetector(mit::tagCodes25h9));
  } else if (tag_family == "16h5") {
    tag_detector_.reset(new mit::TagDetector(mit::tagCodes16h5));
  } else {
    throw std::invalid_argument("Invalid tag family");
  }
}

ApriltagVec ApriltagDetectorMit::DetectImpl(const cv::Mat& image) {
  // Decimate image
  cv::Mat gray;
  if (decimate_ != 1.0) {
    cv::resize(image, gray, cv::Size(0, 0), 1 / decimate_, 1 / decimate_);
  } else {
    gray = image;
  }

  // Detection
  tag_detections_ = tag_detector_->extractTags(gray);
  // Handle empty detection
  if (tag_detections_.empty()) return {};

  // Handle decimation
  if (decimate_ != 1.0) {
    for (mit::TagDetection& td : tag_detections_) {
      td.Scale(decimate_);
    }
  }

  // Refine corners
  if (refine_) RefineDetections(gray);

  return TagDetectionsToApriltagMsg();
}

void ApriltagDetectorMit::DrawImpl(cv::Mat& image) const {
  if (tag_detections_.empty()) return;
  for (const mit::TagDetection& td : tag_detections_) {
    td.draw(image);
  }
}

ApriltagVec ApriltagDetectorMit::TagDetectionsToApriltagMsg() const {
  ApriltagVec apriltags;
  for (const mit::TagDetection& td : tag_detections_) {
    apriltag_msgs::Apriltag apriltag;
    apriltag.id = td.id;
    apriltag.center.x = td.cxy.first;
    apriltag.center.y = td.cxy.second;
    apriltag.hamming = td.hammingDistance;
    apriltag.family = tag_family_;
    apriltag.size = tag_size_;
    for (const auto& p : td.p) {
      geometry_msgs::Point corner;
      corner.x = p.first;
      corner.y = p.second;
      corner.z = 1;
      apriltag.corners.push_back(corner);
    }
    apriltags.push_back(apriltag);
  }
  return apriltags;
}

void ApriltagDetectorMit::RefineDetections(const cv::Mat& image) {
  std::vector<cv::Point2f> corners;
  assert(!tag_detections_.empty());
  for (const mit::TagDetection& td : tag_detections_) {
    for (const auto& p : td.p) {
      corners.push_back(cv::Point2f(p.first, p.second));
    }
  }
  const auto criteria =
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
  cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  decltype(corners.size()) i = 0;
  for (mit::TagDetection& td : tag_detections_) {
    decltype(td.cxy.first) sum_x{0.0}, sum_y{0.0};
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

/// ApriltagDetectorUmich
}  // namespace apriltag_ros
