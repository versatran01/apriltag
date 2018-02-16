#include "apriltag_ros/apriltag_detector.h"

#include <boost/make_shared.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

namespace mit = apriltag_mit;
namespace umich = apriltag_umich;

/// ================
/// ApriltagDetector
/// ================
ApriltagDetector::ApriltagDetector(const DetectorType &detector_type,
                                   const TagFamily &tag_family)
    : payload_(TagFamilyToPayload(tag_family)), detector_type_(detector_type),
      tag_family_(tag_family),
      tag_family_str_(DetectorTypeToString(detector_type)) {}

void ApriltagDetector::set_black_border(int black_border) {
  black_border_ = black_border;
  SetBlackBorder(black_border);
}
int ApriltagDetector::black_border() const { return black_border_; }

void ApriltagDetector::set_decimate(int decimate) { SetDecimate(decimate); }
int ApriltagDetector::decimate() const { return decimate_; }

void ApriltagDetector::set_nthreads(int nthreads) { SetNThreads(nthreads); }
int ApriltagDetector::nthreads() const { return nthreads_; }

int ApriltagDetector::payload() const { return payload_; }
const std::string &ApriltagDetector::tag_family() const {
  return tag_family_str_;
}

ApriltagVec ApriltagDetector::Detect(const cv::Mat &image) {
  if (image.empty())
    return {};

  // Check image type
  cv::Mat gray;
  if (image.type() == CV_8UC1) {
    gray = image;
  } else if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, CV_BGR2GRAY);
  } else {
    return {};
  }

  // Detect
  return DetectImpl(gray);
}

ApriltagDetectorPtr ApriltagDetector::Create(const DetectorType &detector_type,
                                             const TagFamily &tag_family) {
  switch (detector_type) {
  case DetectorType::Mit:
    return boost::make_shared<ApriltagDetectorMit>(tag_family);
  case DetectorType::Umich:
    return ApriltagDetectorPtr(new ApriltagDetectorUmich(tag_family));
  default:
    throw std::invalid_argument("Invalid apriltag detector type.");
  }
}

/// ===================
/// ApriltagDetectorMit
/// ===================
ApriltagDetectorMit::ApriltagDetectorMit(const TagFamily &tag_family)
    : ApriltagDetector(DetectorType::Mit, tag_family) {
  switch (tag_family) {
  case TagFamily::tf36h11:
    tag_detector_ = boost::make_shared<mit::TagDetector>(mit::tag_codes_36h11);
    break;
  case TagFamily::tf25h9:
    tag_detector_ = boost::make_shared<mit::TagDetector>(mit::tag_codes_25h9);
    break;
  case TagFamily::tf16h5:
    tag_detector_ = boost::make_shared<mit::TagDetector>(mit::tag_codes_16h5);
    break;
  default:
    throw std::invalid_argument("Invalid tag family");
  }
}

void ApriltagDetectorMit::SetBlackBorder(int black_border) {
  tag_detector_->set_black_border(black_border);
}
void ApriltagDetectorMit::SetDecimate(int decimate) { decimate_ = 1; }
void ApriltagDetectorMit::SetNThreads(int nthreads) { nthreads_ = 1; }

ApriltagVec ApriltagDetectorMit::DetectImpl(const cv::Mat &image) {
  // Detection
  auto detections = tag_detector_->ExtractTags(image);

  // Convert to  Apriltag message
  ApriltagVec apriltags;
  apriltags.reserve(detections.size());

  for (const mit::TagDetection &td : detections) {
    apriltag_msgs::Apriltag apriltag;
    apriltag.id = td.id;
    apriltag.bits = payload();
    apriltag.border = black_border();
    apriltag.family = tag_family();
    apriltag.hamming = td.hamming_distance;
    apriltag.center.x = td.cxy.x;
    apriltag.center.y = td.cxy.y;
    for (size_t i = 0; i < 4; ++i) {
      apriltag.corners[i].x = td.p[i].x;
      apriltag.corners[i].y = td.p[i].y;
    }
    apriltags.push_back(apriltag);
  }
  return apriltags;
}

/// =====================
/// ApriltagDetectorUmich
/// =====================
ApriltagDetectorUmich::ApriltagDetectorUmich(const TagFamily &tag_family)
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
void ApriltagDetectorUmich::SetDecimate(int decimate) {
  tag_detector_->quad_decimate = decimate;
}
void ApriltagDetectorUmich::SetNThreads(int nthreads) {
  tag_detector_->nthreads = nthreads_;
}

ApriltagVec ApriltagDetectorUmich::DetectImpl(const cv::Mat &image) {
  umich::ImageU8Ptr image_u8(
      image_u8_create_from_gray(image.cols, image.rows, image.data));
  // Detection
  umich::ZarrayPtr detections(
      apriltag_detector_detect(tag_detector_.get(), image_u8.get()));

  // Handle empty detection
  const auto num_detections = zarray_size(detections.get());

  ApriltagVec apriltags;
  apriltags.reserve(num_detections);

  for (int i = 0; i < num_detections; ++i) {
    apriltag_detection_t *td;
    zarray_get(detections.get(), i, &td);
    apriltag_msgs::Apriltag apriltag;
    apriltag.id = td->id;
    apriltag.bits = payload();
    apriltag.hamming = td->hamming;
    apriltag.border = black_border();
    apriltag.center.x = td->c[0];
    apriltag.center.y = td->c[1];
    for (size_t i = 0; i < 4; ++i) {
      // Umich's order of corners is different from mit's
      apriltag.corners[i].x = td->p[i][0];
      apriltag.corners[i].y = td->p[i][1];
    }
    apriltags.push_back(apriltag);
  }
  return apriltags;
}

void DrawApriltag(cv::Mat &image, const apriltag_msgs::Apriltag &apriltag,
                  int thickness, bool draw_corners) {
  const auto &p = apriltag.corners;
  cv::line(image, cv::Point2i(p[0].x, p[0].y), cv::Point2i(p[1].x, p[1].y),
           CV_RGB(255, 0, 0), thickness);
  cv::line(image, cv::Point2i(p[0].x, p[0].y), cv::Point2i(p[3].x, p[3].y),
           CV_RGB(0, 255, 0), thickness);
  cv::line(image, cv::Point2i(p[2].x, p[2].y), cv::Point2i(p[3].x, p[3].y),
           CV_RGB(0, 0, 255), thickness);
  cv::line(image, cv::Point2i(p[2].x, p[2].y), cv::Point2i(p[1].x, p[1].y),
           CV_RGB(0, 0, 255), thickness);

  if (draw_corners) {
    int r = thickness;
    cv::circle(image, cv::Point2i(p[0].x, p[0].y), r, CV_RGB(255, 0, 0), -1,
               CV_AA);
    cv::circle(image, cv::Point2i(p[1].x, p[1].y), r, CV_RGB(0, 255, 0), -1,
               CV_AA);
    cv::circle(image, cv::Point2i(p[2].x, p[2].y), r, CV_RGB(0, 0, 255), -1,
               CV_AA);
    cv::circle(image, cv::Point2i(p[3].x, p[3].y), r, CV_RGB(255, 0, 255), -1,
               CV_AA);
  }

  cv::putText(image, std::to_string(apriltag.id),
              cv::Point2f(apriltag.center.x - 5, apriltag.center.y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 255), 2, CV_AA);
}

void DrawApriltags(cv::Mat &image, const ApriltagVec &apriltags) {
  for (const auto &apriltag : apriltags) {
    DrawApriltag(image, apriltag);
  }
}

int TagFamilyToPayload(const TagFamily &tag_family) {
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

std::string DetectorTypeToString(const DetectorType &detector_type) {
  switch (detector_type) {
  case DetectorType::Mit:
    return {"mit"};
  case DetectorType::Umich:
    return {"umich"};
  default:
    throw std::invalid_argument("Invalid detector type");
  }
}

bool InsideImage(const cv::Mat &image, float x, float y, int b) {
  const auto w = image.cols;
  const auto h = image.rows;
  return (x >= b) && (y >= b) && (x < w - b) && (y < h - b);
}

bool InsideImage(const cv::Mat &image, const cv::Point2f &p, int b) {
  return InsideImage(image, p.x, p.y, b);
}

// void RefineApriltags(const cv::Mat &image, ApriltagVec &apriltags,
//                     int win_size) {
//  if (apriltags.empty())
//    return;

//  std::vector<cv::Point2f> corners;
//  corners.reserve(apriltags.size() * 4);
//  for (const auto &apriltag : apriltags) {
//    for (const auto &corner : apriltag.corners) {
//      if (InsideImage(image, corner.x, corner.y, win_size)) {
//        corners.push_back(cv::Point2f(corner.x, corner.y));
//      }
//    }
//  }

//  const auto cv_win_size = cv::Size(win_size, win_size);
//  const auto zero_zone = cv::Size(-1, -1);
//  const auto criteria =
//      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001);

//  cv::cornerSubPix(image, corners, cv_win_size, zero_zone, criteria);

//  size_t i = 0;
//  for (auto &apriltag : apriltags) {
//    for (auto &corner : apriltag.corners) {
//      if (InsideImage(image, corner.x, corner.y, win_size)) {
//        const auto &refined = corners[i++];
//        corner.x = refined.x;
//        corner.y = refined.y;
//      }
//    }
//  }
//}

} // namespace apriltag_ros
