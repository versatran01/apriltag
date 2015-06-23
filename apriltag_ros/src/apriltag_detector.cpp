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
    : detector_type_(type), tag_family_(tag_family) {}

void ApriltagDetector::detect(const cv::Mat& image) {
  if (image.empty()) return;
  // Cleanup previous detections
  apriltags_.clear();
  // Check image type
  cv::Mat gray;
  if (image.type() == CV_8UC1) {
    gray = image;
  } else if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, CV_BGR2GRAY);
  }
  // Detect
  detectImpl(gray);
}

void ApriltagDetector::draw(cv::Mat& image) const {
  for (const apriltag_msgs::Apriltag& apriltag : apriltags_) {
    drawApriltag(image, apriltag);
  }
}

ApriltagDetectorPtr ApriltagDetector::create(const std::string& type,
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
  if (tag_family == "t36h11") {
    tag_detector_.reset(new mit::TagDetector(mit::tagCodes36h11));
    tag_bits_ = 6;
  } else if (tag_family == "t25h9") {
    tag_detector_.reset(new mit::TagDetector(mit::tagCodes25h9));
    tag_bits_ = 5;
  } else if (tag_family == "t16h5") {
    tag_detector_.reset(new mit::TagDetector(mit::tagCodes16h5));
    tag_bits_ = 4;
  } else {
    throw std::invalid_argument("Invalid tag family");
  }
}

void ApriltagDetectorMit::detectImpl(const cv::Mat& image) {
  // Decimate image
  cv::Mat im_scaled;
  if (decimate_ != 1) {
    cv::resize(image, im_scaled, cv::Size(0, 0), 1.0 / decimate_,
               1.0 / decimate_);
  } else {
    im_scaled = image;
  }

  // Detection
  std::vector<mit::TagDetection> detections =
      tag_detector_->extractTags(im_scaled);
  // Handle empty detection
  if (detections.empty()) return;

  // Handle decimation
  if (decimate_ != 1) {
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
    apriltag_msgs::Apriltag apriltag;
    apriltag.id = td.id;
    apriltag.bits = tag_bits();
    apriltag.border = black_border();
    apriltag.family = tag_family();
    apriltag.hamming = td.hammingDistance;
    apriltag.center.x = td.cxy.first;
    apriltag.center.y = td.cxy.second;
    for (size_t i = 0; i < 4; ++i) {
      apriltag.corners[i].x = td.p[i].first;
      apriltag.corners[i].y = td.p[i].second;
    }
    apriltags_.push_back(apriltag);
  }
}

/// =====================
/// ApriltagDetectorUmich
/// =====================
ApriltagDetectorUmich::ApriltagDetectorUmich(const std::string& tag_family)
    : ApriltagDetector("umich", tag_family),
      tag_detector_(apriltag_detector_create()) {
  if (tag_family == "t36h11") {
    tag_family_.reset(tag36h11_create());
    tag_bits_ = 6;
  } else if (tag_family == "t25h9") {
    tag_family_.reset(tag25h9_create());
    tag_bits_ = 5;
  } else if (tag_family == "t16h5") {
    tag_family_.reset(tag16h5_create());
    tag_bits_ = 4;
  } else {
    throw std::invalid_argument("Invalid tag family");
  }
  apriltag_detector_add_family(tag_detector_.get(), tag_family_.get());
}

void ApriltagDetectorUmich::detectImpl(const cv::Mat& image) {
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
    apriltag_msgs::Apriltag apriltag;
    apriltag.id = td->id;
    apriltag.bits = tag_bits();
    apriltag.hamming = td->hamming;
    apriltag.border = black_border();
    apriltag.center.x = td->c[0];
    apriltag.center.y = td->c[1];
    for (size_t i = 0; i < 4; ++i) {
      apriltag.corners[i].x = td->p[3 - i][0];
      apriltag.corners[i].y = td->p[3 - i][1];
    }
    apriltags_.push_back(apriltag);
  }
}

void drawApriltag(cv::Mat& image, const apriltag_msgs::Apriltag& apriltag,
                  int thickness) {
  const auto& p = apriltag.corners;
  cv::line(image, cv::Point2i(p[0].x, p[0].y), cv::Point2i(p[1].x, p[1].y),
           CV_RGB(255, 0, 0), thickness);
  cv::line(image, cv::Point2i(p[0].x, p[0].y), cv::Point2i(p[3].x, p[3].y),
           CV_RGB(0, 255, 0), thickness);
  cv::line(image, cv::Point2i(p[2].x, p[2].y), cv::Point2i(p[3].x, p[3].y),
           CV_RGB(0, 0, 255), thickness);
  cv::line(image, cv::Point2i(p[2].x, p[2].y), cv::Point2i(p[1].x, p[1].y),
           CV_RGB(0, 0, 255), thickness);

  cv::putText(image, std::to_string(apriltag.id),
              cv::Point2f(apriltag.center.x - 5, apriltag.center.y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 255), 2);
}

}  // namespace apriltag_ros
