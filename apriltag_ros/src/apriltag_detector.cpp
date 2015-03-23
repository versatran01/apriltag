#include "apriltag_ros/apriltag_detector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CV_RED CV_RGB(255, 0, 0)
#define CV_GREEN CV_RGB(0, 255, 0)
#define CV_BLUE CV_RGB(0, 0, 255)
#define CV_MAGENTA CV_RGB(255, 0, 255)

namespace apriltag_ros {

namespace mit = apriltag_mit;
namespace umich = apriltag_umich;

/// ================
/// ApriltagDetector
/// ================
ApriltagDetector::ApriltagDetector(const std::string& type,
                                   const std::string& tag_family)
    : type_(type), tag_family_(tag_family) {}

ApriltagVec ApriltagDetector::Detect(const cv::Mat& image) {
  if (image.empty()) return {};
  return DetectImpl(image);
}

void ApriltagDetector::Draw(cv::Mat& image) const { DrawImpl(image); }

ApriltagDetectorPtr ApriltagDetector::Create(const std::string& type,
                                             const std::string& tag_family) {
  if (type == "mit") {
    return ApriltagDetectorPtr(new ApriltagDetectorMit(tag_family));
  } else if (type == "umich") {
    return ApriltagDetectorPtr(new ApriltagDetectorUmich(tag_family));
  } else {
    throw std::invalid_argument("Invalid apriltag detector type.");
  }
}

/// ===================
/// ApriltagDetectorMit
/// ===================
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
  for (const mit::TagDetection& det : tag_detections_) {
    apriltag_msgs::Apriltag apriltag;
    apriltag.id = det.id;
    apriltag.center.x = det.cxy.first;
    apriltag.center.y = det.cxy.second;
    apriltag.hamming = det.hammingDistance;
    apriltag.family = tag_family();
    apriltag.size = tag_size();
    for (const mit::FloatPair& p : det.p) {
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
    for (const mit::FloatPair& p : td.p) {
      corners.push_back(cv::Point2f(p.first, p.second));
    }
  }
  const auto criteria =
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
  cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

  decltype(corners.size()) i{0};
  for (mit::TagDetection& td : tag_detections_) {
    decltype(td.cxy.first) sum_x{0.0}, sum_y{0.0};
    for (mit::FloatPair& p : td.p) {
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

/// =====================
/// ApriltagDetectorUmich
/// =====================
ApriltagDetectorUmich::ApriltagDetectorUmich(const std::string& tag_family)
    : ApriltagDetector("umich", tag_family),
      tag_detector_(apriltag_detector_create()) {
  if (tag_family == "36h11") {
    tag_family_.reset(tag36h11_create());
  } else if (tag_family == "25h9") {
    tag_family_.reset(tag25h9_create());
  } else if (tag_family == "16h5") {
    tag_family_.reset(tag16h5_create());
  } else {
    throw std::invalid_argument("Invalid tag family");
  }
  apriltag_detector_add_family(tag_detector_.get(), tag_family_.get());
}

ApriltagVec ApriltagDetectorUmich::DetectImpl(const cv::Mat& image) {
  umich::ImageU8Ptr image_u8(
      image_u8_create_from_gray(image.cols, image.rows, image.data));
  // Handle options
  tag_detector_->quad_decimate = decimate_;
  tag_detector_->refine_edges = refine_;

  // Detection
  tag_detections_.reset(
      apriltag_detector_detect(tag_detector_.get(), image_u8.get()));

  // Handle empty detection
  const auto num_detections = zarray_size(tag_detections_.get());
  if (num_detections == 0) return {};

  return TagDetectionsToApriltagMsg();
}

ApriltagVec ApriltagDetectorUmich::TagDetectionsToApriltagMsg() const {
  ApriltagVec apriltags;
  for (int i = 0; i < zarray_size(tag_detections_.get()); ++i) {
    apriltag_detection_t* det;
    zarray_get(tag_detections_.get(), i, &det);

    apriltag_msgs::Apriltag apriltag;
    apriltag.id = det->id;
    apriltag.family = tag_family();
    apriltag.size = tag_size();
    apriltag.center.x = det->c[0];
    apriltag.center.y = det->c[1];
    apriltag.hamming = det->hamming;
    for (int i = 3; i >= 0; --i) {
      geometry_msgs::Point corner;
      corner.x = det->p[i][0];
      corner.y = det->p[i][1];
      corner.z = 1;
      apriltag.corners.push_back(corner);
    }
    apriltags.push_back(apriltag);
  }

  return apriltags;
}

void ApriltagDetectorUmich::DrawImpl(cv::Mat& image) const {
  if (!tag_detections_) return;
  for (int i = 0; i < zarray_size(tag_detections_.get()); ++i) {
    apriltag_detection_t* det;
    zarray_get(tag_detections_.get(), i, &det);

    int thickness = 1;
    cv::line(image, cv::Point2f(det->p[3][0], det->p[3][1]),
             cv::Point2f(det->p[2][0], det->p[2][1]), CV_RED, thickness);
    cv::line(image, cv::Point2f(det->p[3][0], det->p[3][1]),
             cv::Point2f(det->p[0][0], det->p[0][1]), CV_GREEN, thickness);
    cv::line(image, cv::Point2f(det->p[1][0], det->p[1][1]),
             cv::Point2f(det->p[2][0], det->p[2][1]), CV_BLUE, thickness);
    cv::line(image, cv::Point2f(det->p[0][0], det->p[0][1]),
             cv::Point2f(det->p[1][0], det->p[1][1]), CV_BLUE, thickness);

    cv::putText(image, std::to_string(det->id),
                cv::Point2f(det->c[0] - 5, det->c[1] + 5),
                cv::FONT_HERSHEY_SIMPLEX, 1, CV_MAGENTA, 2);
  }
}

}  // namespace apriltag_ros
