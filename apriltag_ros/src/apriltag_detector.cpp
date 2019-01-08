#include "apriltag_ros/apriltag_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

std::vector<apriltag_msgs::Apriltag> ApriltagDetector::Detect(
    const cv::Mat &gray, int max_hamming) const {
  if (gray.empty()) return {};

  BOOST_ASSERT(gray.type() == CV_8UC1);

  image_u8_t im = {.width = gray.cols,
                   .height = gray.rows,
                   .stride = gray.cols,
                   .buf = gray.data};

  apriltag::DetectionZaUPtr detections(
      apriltag_detector_detect(td_.get(), &im));
  const auto n = zarray_size(detections.get());

  std::vector<Apriltag> tags;

  for (int i = 0; i < n; i++) {
    apriltag_detection_t *det;
    zarray_get(detections.get(), i, &det);

    if (det->hamming > max_hamming) continue;

    Apriltag tag;
    tag.id = det->id;
    tag.bits = det->family->nbits;
    tag.hamming = det->hamming;
    tag.family = std::string(det->family->name);
    tag.center.x = det->c[0];
    tag.center.y = det->c[1];

    for (size_t i = 0; i < 4; ++i) {
      tag.corners[i].x = det->p[i][0];
      tag.corners[i].y = det->p[i][1];
    }

    tags.push_back(tag);
  }

  return tags;
}

apriltag::FamilyUPtr ApriltagDetector::MakeFamily(
    const ApriltagFamily &family) const {
  apriltag::FamilyUPtr tf;
  switch (family) {
    case ApriltagFamily::tag36h11:
      tf.reset(tag36h11_create());
      break;
    case ApriltagFamily::tag25h9:
      tf.reset(tag25h9_create());
      break;
    case ApriltagFamily::tag16h5:
      tf.reset(tag16h5_create());
      break;
    default:
      throw std::invalid_argument("Invalid apriltag family");
  }
  return tf;
}

bool ApriltagDetector::AddFamily(const ApriltagFamily &family) {
  if (tfs_.find(family) != tfs_.end()) return false;
  tfs_[family] = MakeFamily(family);
  apriltag_detector_add_family(td_.get(), tfs_[family].get());
  return true;
}

bool ApriltagDetector::RemoveFamily(const ApriltagFamily &family) {
  const auto it = tfs_.find(family);
  if (it == tfs_.end()) return false;
  apriltag_detector_remove_family(td_.get(), it->second.get());
  tfs_.erase(it);
  return true;
}

void ApriltagDetector::ClearFamily() {
  apriltag_detector_clear_families(td_.get());
  tfs_.clear();
}

void DrawApriltag(cv::Mat &image, const Apriltag &tag, int thickness,
                  bool draw_corners, bool draw_id) {
  const auto line = cv::LINE_AA;
  const auto &p = tag.corners;
  cv::line(image, cv::Point(p[0].x, p[0].y), cv::Point(p[1].x, p[1].y),
           CV_RGB(255, 0, 0), thickness, line);
  cv::line(image, cv::Point(p[0].x, p[0].y), cv::Point(p[3].x, p[3].y),
           CV_RGB(0, 255, 0), thickness, line);
  cv::line(image, cv::Point(p[2].x, p[2].y), cv::Point(p[3].x, p[3].y),
           CV_RGB(0, 0, 255), thickness, line);
  cv::line(image, cv::Point(p[2].x, p[2].y), cv::Point(p[1].x, p[1].y),
           CV_RGB(0, 0, 255), thickness, line);

  if (draw_corners) {
    int r = thickness;
    cv::circle(image, cv::Point(p[0].x, p[0].y), r, CV_RGB(255, 0, 0), -1,
               line);
    cv::circle(image, cv::Point(p[1].x, p[1].y), r, CV_RGB(0, 255, 0), -1,
               line);
    cv::circle(image, cv::Point(p[2].x, p[2].y), r, CV_RGB(0, 0, 255), -1,
               line);
    cv::circle(image, cv::Point(p[3].x, p[3].y), r, CV_RGB(255, 0, 255), -1,
               line);
  }

  if (draw_id) {
    int baseline;
    const double fontscale{1.0};
    const int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    const auto text = std::to_string(tag.id);
    const auto textsize =
        cv::getTextSize(text, fontface, 1.0, thickness, &baseline);
    cv::putText(image, text,
                cv::Point(tag.center.x - textsize.width / 2,
                          tag.center.y + textsize.height / 2),
                fontface, fontscale, CV_RGB(255, 0, 255), thickness, line);
  }
}

void DrawApriltags(cv::Mat &image, const std::vector<Apriltag> &apriltags,
                   int thickness, bool draw_corners, bool draw_id) {
  for (const auto &apriltag : apriltags) {
    DrawApriltag(image, apriltag, thickness, draw_corners, draw_id);
  }
}

}  // namespace apriltag_ros
