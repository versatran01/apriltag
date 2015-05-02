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
  tag_detections_.clear();
  // Check image type
  cv::Mat gray;
  if (image.type() == CV_8UC1) {
    gray = image;
  } else if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, CV_BGR2GRAY);
  }
  // Detect
  detectImpl(gray);
  // Set width and height of image
  for (auto& td : tag_detections_) {
    td.w = image.cols;
    td.h = image.rows;
  }
}

void ApriltagDetector::estimate(const cv::Matx33d& K,
                                const cv::Mat_<double>& D) {
  for (ApriltagDetection& td : tag_detections_) {
    td.estimate(tag_size(), K, D);
  }
}

void ApriltagDetector::draw(cv::Mat& image) const {
  for (const ApriltagDetection& td : tag_detections_) {
    td.draw(image);
  }
}

void ApriltagDetector::zoom(const cv::Mat& image, cv::Mat& view, int win_size,
                            int tags_per_row, bool draw_grid) const {
  if (empty() || image.empty()) return;
  const int k = 2;    // arrange corners in a 2x2 grid
  auto s = win_size;  // this is actually half win size
  // Make sure s is odd
  s = (s % 2 == 0) ? s + 1 : s;
  const auto s2 = s * 2 + 1;  // this is full win size
  const auto tag_cols = tags_per_row;
  const auto tag_rows = (size() - 1) / tag_cols + 1;

  view = cv::Mat::zeros(tag_rows * k * s2, tag_cols * 2 * s2, CV_8UC3);
  // Pad the image
  cv::Mat image_pad;
  cv::copyMakeBorder(image, image_pad, s, s, s, s, cv::BORDER_CONSTANT);
  for (size_t i = 0; i < size(); ++i) {
    // for each tag
    // calculate the row and col of this tag
    int r = i / tag_cols;
    int c = i - r * tag_cols;
    const ApriltagDetection& td = tag_detections_[i];
    for (int j = 0; j < 4; ++j) {
      // for each corner
      int ix = td.p[j][0];
      int iy = td.p[j][1];
      // Make sure in bound
      if (!isInsideImage(ix, iy, image.cols, image.rows)) continue;
      int x_dst = (c * k + j - (j / k) * k) * s2;
      int y_dst = (r * k + j / 2) * s2;
      cv::Mat src(image_pad(cv::Rect(ix, iy, s2, s2)));
      cv::Mat dst(view(cv::Rect(x_dst, y_dst, s2, s2)));
      src.copyTo(dst);
    }
  }
  if (draw_grid) {
    drawGrid(view, tag_rows * 2, tag_cols * 2, s2, CV_RGB(0, 255, 0));
    drawGrid(view, tag_rows, tag_cols, s2 * 2, CV_RGB(255, 0, 0));
  }
}

ApriltagVec ApriltagDetector::toApriltagMsg() const {
  ApriltagVec apriltags;
  for (const auto& td : tag_detections_) {
    auto apriltag = static_cast<apriltag_msgs::Apriltag>(td);
    apriltag.size = tag_size();
    apriltag.bits = tag_bits();
    apriltag.family = tag_family();
    apriltag.border = black_border();
    apriltags.push_back(apriltag);
  }
  return apriltags;
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
    tag_detections_.push_back(ApriltagDetection(td));
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
    tag_detections_.push_back(ApriltagDetection(td));
  }
}

void drawGrid(cv::Mat& image, int rows, int cols, int size,
              const cv::Scalar& color) {
  for (int c = 1; c < cols; ++c) {
    int x = c * size;
    cv::line(image, {x, 0}, {x, image.rows}, color);
  }
  for (int r = 1; r < rows; ++r) {
    int y = r * size;
    cv::line(image, {0, y}, {image.cols, y}, color);
  }
}

}  // namespace apriltag_ros
