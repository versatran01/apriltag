#include "apriltag_ros/apriltag_detector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_ros {

namespace mit = apriltag_mit;
namespace umich = apriltag_umich;

/// ================
/// ApriltagDetector
/// ================
ApriltagDetector::ApriltagDetector(const std::string& type,
                                   const std::string& tag_family)
    : type_(type), tag_family_(tag_family) {}

void ApriltagDetector::Detect(const cv::Mat& image) {
  if (image.empty()) return;
  // Cleanup previous detections
  tag_detections_.clear();
  // Check image type
  cv::Mat gray;
  if (image.type() == CV_8UC1) {
    gray = image;
  } else if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, CV_BGR2GRAY);
  }
  // Detect
  DetectImpl(gray);
}

void ApriltagDetector::Estimate(/* K, D */) {
  // TODO: just call ApriltagDetection::Estimate()
  if (tag_size() == 0) return;
}

void ApriltagDetector::Draw(cv::Mat& image) const {
  for (const ApriltagDetection& td : tag_detections_) {
    td.Draw(image);
  }
}

ApriltagVec ApriltagDetector::ToApriltagMsg() const {
  ApriltagVec apriltags;
  for (const ApriltagDetection& td : tag_detections_) {
    apriltag_msgs::Apriltag apriltag = static_cast<apriltag_msgs::Apriltag>(td);
    apriltag.size = tag_size();
    apriltag.family = tag_family();
    apriltags.push_back(apriltag);
  }
  return apriltags;
}

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

void ApriltagDetectorMit::DetectImpl(const cv::Mat& image) {
  // Decimate image
  cv::Mat im_scaled;
  if (decimate_ != 1.0) {
    cv::resize(image, im_scaled, cv::Size(0, 0), 1 / decimate_, 1 / decimate_);
  } else {
    im_scaled = image;
  }

  // Detection
  std::vector<mit::TagDetection> detections =
      tag_detector_->extractTags(im_scaled);
  // Handle empty detection
  if (detections.empty()) return;

  // Handle decimation
  if (decimate_ != 1.0) {
    for (mit::TagDetection& td : detections) {
      td.scaleTag(decimate_);
    }
  }

  // Refine corners
  // Disable for now, maybe add black squares at corner
  //  if (refine_) {
  //    for (mit::TagDetection& td : detections) {
  //      td.refineTag(im_scaled);
  //    }
  //  }

  // Convert to common ApriltagDetection type
  for (const mit::TagDetection& td : detections) {
    tag_detections_.push_back(ApriltagDetection(td));
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

void ApriltagDetectorUmich::DetectImpl(const cv::Mat& image) {
  umich::ImageU8Ptr image_u8(
      image_u8_create_from_gray(image.cols, image.rows, image.data));
  // Handle options
  tag_detector_->quad_decimate = decimate_;
  tag_detector_->refine_edges = refine_;

  // Detection
  umich::ZarrayPtr detections(
      apriltag_detector_detect(tag_detector_.get(), image_u8.get()));

  // Handle empty detection
  const auto num_detections = zarray_size(detections.get());
  if (num_detections == 0) return;

  for (int i = 0; i < num_detections; ++i) {
    apriltag_detection_t* td;
    zarray_get(detections.get(), i, &td);
    tag_detections_.push_back(ApriltagDetection(td));
  }
}

}  // namespace apriltag_ros
