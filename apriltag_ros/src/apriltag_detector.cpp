#include "apriltag_ros/apriltag_detector.h"

#include <boost/make_shared.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_ros {

namespace mit = apriltag_mit;
namespace umich = apriltag_umich;

/// ================
/// ApriltagDetector
/// ================
ApriltagDetector::ApriltagDetector(const DetectorType& detector_type,
                                   const TagFamily& tag_family)
    : tag_bits_(TagFamilyToTagBits(tag_family)),
      detector_type_(detector_type),
      tag_family_(tag_family),
      tag_family_str_(DetectorTypeToString(detector_type)) {}

void ApriltagDetector::set_black_border(int black_border) {
  black_border_ = black_border;
  SetBlackBorder(black_border);
}

int ApriltagDetector::black_border() const { return black_border_; }

void ApriltagDetector::set_tag_bits(int tag_bits) { tag_bits_ = tag_bits; }
int ApriltagDetector::tag_bits() const { return tag_bits_; }

void ApriltagDetector::set_decimate(int decimate) {
  decimate_ = (decimate >= 1) ? decimate : 1;
}
int ApriltagDetector::decimate() const { return decimate_; }

void ApriltagDetector::set_refine(bool refine) { refine_ = refine; }
bool ApriltagDetector::refine() const { return refine_; }

const std::string& ApriltagDetector::tag_family() const {
  return tag_family_str_;
}

const ApriltagVec& ApriltagDetector::apriltags() const { return apriltags_; }

void ApriltagDetector::Detect(const cv::Mat& image) {
  if (image.empty()) return;
  // Cleanup previous detections
  apriltags_.clear();
  // Check image type
  cv::Mat gray;
  if (image.type() == CV_8UC1) {
    gray = image;
  } else if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, CV_BGR2GRAY);
  } else {
    return;
  }
  // Detect
  DetectImpl(gray);
}

void ApriltagDetector::Draw(cv::Mat& image) const {
  for (const auto& apriltag : apriltags_) {
    DrawApriltag(image, apriltag);
  }
}

ApriltagDetectorPtr ApriltagDetector::Create(const DetectorType& detector_type,
                                             const TagFamily& tag_family) {
  switch (detector_type) {
    case DetectorType::Mit:
      return boost::make_shared<ApriltagDetectorMit>(tag_family);
    case DetectorType::Umich:
      // TODO: Change this to boost::shared_ptr
      return ApriltagDetectorPtr(new ApriltagDetectorUmich(tag_family));
    default:
      throw std::invalid_argument("Invalid apriltag detector type.");
  }
}

/// ===================
/// ApriltagDetectorMit
/// ===================
ApriltagDetectorMit::ApriltagDetectorMit(const TagFamily& tag_family)
    : ApriltagDetector(DetectorType::Mit, tag_family) {
  switch (tag_family) {
    case TagFamily::tf36h11:
      tag_detector_ = boost::make_shared<mit::TagDetector>(mit::tagCodes36h11);
      break;
    case TagFamily::tf25h9:
      tag_detector_ = boost::make_shared<mit::TagDetector>(mit::tagCodes25h9);
      break;
    case TagFamily::tf16h5:
      tag_detector_ = boost::make_shared<mit::TagDetector>(mit::tagCodes16h5);
      break;
    default:
      throw std::invalid_argument("Invalid tag family");
  }
}

void ApriltagDetectorMit::SetBlackBorder(int black_border) {
  tag_detector_->setBlackBorder(black_border);
}

void ApriltagDetectorMit::DetectImpl(const cv::Mat& image) {
  // Decimate image
  cv::Mat im_scaled;
  if (decimate_ > 1) {
    const double fx = 1.0 / decimate_;
    cv::resize(image, im_scaled, cv::Size(0, 0), fx, fx);
  } else {
    im_scaled = image;
  }

  // Detection
  auto detections = tag_detector_->extractTags(im_scaled);

  // Handle decimation
  if (decimate_ > 1) {
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

  // Convert to  Apriltag message
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
ApriltagDetectorUmich::ApriltagDetectorUmich(const TagFamily& tag_family)
    : ApriltagDetector(DetectorType::Umich, tag_family),
      tag_detector_(apriltag_detector_create()) {
  switch (tag_family) {
    case TagFamily::tf36h11:
      tag_family_.reset(tag36h11_create());
      break;
    case TagFamily::tf25h9:
      tag_family_.reset(tag25h9_create());
      break;
    case TagFamily::tf16h5:
      tag_family_.reset(tag16h5_create());
      break;
    default:
      throw std::invalid_argument("Invalid tag family");
  }
  apriltag_detector_add_family(tag_detector_.get(), tag_family_.get());
}

void ApriltagDetectorUmich::SetBlackBorder(int black_border) {
  tag_family_->black_border = black_border;
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

void DrawApriltag(cv::Mat& image, const apriltag_msgs::Apriltag& apriltag,
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

int TagFamilyToTagBits(const TagFamily& tag_family) {
  switch (tag_family) {
    case TagFamily::tf36h11:
      return 6;
    case TagFamily::tf25h9:
      return 5;
    case TagFamily::tf16h5:
      return 4;
    default:
      throw std::invalid_argument("Invalid tag family");
  }
}

std::string DetectorTypeToString(const DetectorType& detector_type) {
  switch (detector_type) {
    case DetectorType::Mit:
      return std::string("mit");
    case DetectorType::Umich:
      return std::string("umich");
    default:
      throw std::invalid_argument("Invalid detector type");
  }
}

}  // namespace apriltag_ros
