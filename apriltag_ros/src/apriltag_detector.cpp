#include "apriltag_ros/apriltag_detector.h"

#include <boost/assert.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag_ros {

// ApriltagDetector::ApriltagDetector(
//    const std::vector<ApriltagFamily> &families) {
//  for (const auto &family : families) {
//    AddFamily(family);
//  }
//}

void ApriltagDetector::Detect(const cv::Mat &gray) {
  if (gray.empty()) return;

  // Check image type
  BOOST_ASSERT_MSG(gray.type() == CV_8UC1, "image is not grayscale");
  cv::Mat frame;
  cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);

  // Detect
  image_u8_t im = {.width = gray.cols,
                   .height = gray.rows,
                   .stride = gray.cols,
                   .buf = gray.data};

  apriltag::ZarrayUPtr detections(apriltag_detector_detect(td_.get(), &im));
  std::cout << zarray_size(detections.get()) << " tags detected" << std::endl;

  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections.get()); i++) {
    apriltag_detection_t *det;
    zarray_get(detections.get(), i, &det);
    cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[1][0], det->p[1][1]), CV_RGB(255, 0, 0), 2);
    cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[3][0], det->p[3][1]), CV_RGB(0, 255, 0), 2);
    cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
             cv::Point(det->p[2][0], det->p[2][1]), CV_RGB(0, 0, 255), 2);
    cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
             cv::Point(det->p[3][0], det->p[3][1]), CV_RGB(255, 0, 255), 2);

    std::stringstream ss;
    ss << det->id;
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    cv::Size textsize =
        cv::getTextSize(text, fontface, fontscale, 2, &baseline);
    cv::putText(frame, text,
                cv::Point(det->c[0] - textsize.width / 2,
                          det->c[1] + textsize.height / 2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
  }
  cv::imshow("frame", frame);
  cv::waitKey(1);
}

apriltag::FamilyUPtr ApriltagDetector::MakeFamily(
    const ApriltagFamily &family) const {
  apriltag::FamilyUPtr tf;
  switch (family) {
    case ApriltagFamily::tf36h11:
      tf.reset(tag36h11_create());
      break;
    case ApriltagFamily::tf25h9:
      tf.reset(tag25h9_create());
      break;
    case ApriltagFamily::tf16h5:
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

}  // namespace apriltag_ros

// ApriltagVec ApriltagDetectorUmich::DetectImpl(const cv::Mat &image) {
//  umich::ImageU8Ptr image_u8(
//      image_u8_create_from_gray(image.cols, image.rows, image.data));
//  // Detection
//  umich::ZarrayPtr detections(
//      apriltag_detector_detect(tag_detector_.get(), image_u8.get()));

//  // Handle empty detection
//  const auto num_detections = zarray_size(detections.get());

//  ApriltagVec apriltags;
//  apriltags.reserve(num_detections);

//  for (int i = 0; i < num_detections; ++i) {
//    apriltag_detection_t *td;
//    zarray_get(detections.get(), i, &td);
//    apriltag_msgs::Apriltag apriltag;
//    apriltag.id = td->id;
//    apriltag.bits = payload();
//    apriltag.hamming = td->hamming;
//    apriltag.border = black_border();
//    apriltag.center.x = td->c[0];
//    apriltag.center.y = td->c[1];
//    for (size_t i = 0; i < 4; ++i) {
//      // Umich's order of corners is different from mit's
//      apriltag.corners[i].x = td->p[i][0];
//      apriltag.corners[i].y = td->p[i][1];
//    }
//    apriltags.push_back(apriltag);
//  }
//  return apriltags;
//}

// void DrawApriltag(cv::Mat &image, const apriltag_msgs::Apriltag &apriltag,
//                  int thickness, bool draw_corners) {
//  const auto &p = apriltag.corners;
//  cv::line(image, cv::Point2i(p[0].x, p[0].y), cv::Point2i(p[1].x, p[1].y),
//           CV_RGB(255, 0, 0), thickness);
//  cv::line(image, cv::Point2i(p[0].x, p[0].y), cv::Point2i(p[3].x, p[3].y),
//           CV_RGB(0, 255, 0), thickness);
//  cv::line(image, cv::Point2i(p[2].x, p[2].y), cv::Point2i(p[3].x, p[3].y),
//           CV_RGB(0, 0, 255), thickness);
//  cv::line(image, cv::Point2i(p[2].x, p[2].y), cv::Point2i(p[1].x, p[1].y),
//           CV_RGB(0, 0, 255), thickness);

//  const auto line_type = cv::LINE_AA;
//  if (draw_corners) {
//    int r = thickness;
//    cv::circle(image, cv::Point2i(p[0].x, p[0].y), r, CV_RGB(255, 0, 0), -1,
//               line_type);
//    cv::circle(image, cv::Point2i(p[1].x, p[1].y), r, CV_RGB(0, 255, 0), -1,
//               line_type);
//    cv::circle(image, cv::Point2i(p[2].x, p[2].y), r, CV_RGB(0, 0, 255), -1,
//               line_type);
//    cv::circle(image, cv::Point2i(p[3].x, p[3].y), r, CV_RGB(255, 0, 255), -1,
//               line_type);
//  }

//  cv::putText(image, std::to_string(apriltag.id),
//              cv::Point2f(apriltag.center.x - 5, apriltag.center.y + 5),
//              cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 255), 2, line_type);
//}

// void DrawApriltags(cv::Mat &image, const ApriltagVec &apriltags) {
//  for (const auto &apriltag : apriltags) {
//    DrawApriltag(image, apriltag);
//  }
//}

// int TagFamilyToPayload(const TagFamily &tag_family) {
//  switch (tag_family) {
//    case TagFamily::tf36h11:
//      return 6;
//    case TagFamily::tf25h9:
//      return 5;
//    case TagFamily::tf16h5:
//      return 4;
//    default:
//      throw std::invalid_argument("Invalid tag family");
//  }
//}

// std::string DetectorTypeToString(const DetectorType &detector_type) {
//  switch (detector_type) {
//    case DetectorType::Mit:
//      return {"mit"};
//    case DetectorType::Umich:
//      return {"umich"};
//    default:
//      throw std::invalid_argument("Invalid detector type");
//  }
//}

// bool InsideImage(const cv::Mat &image, float x, float y, int b) {
//  const auto w = image.cols;
//  const auto h = image.rows;
//  return (x >= b) && (y >= b) && (x < w - b) && (y < h - b);
//}

// bool InsideImage(const cv::Mat &image, const cv::Point2f &p, int b) {
//  return InsideImage(image, p.x, p.y, b);
//}
