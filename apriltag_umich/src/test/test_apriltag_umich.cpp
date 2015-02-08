#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/package.h>

#include "apriltag_umich/apriltag_umich.h"

// Detect April tags
// april_tag_family_t *tf = tag36h11_create();
// april_tag_detector_t *td = april_tag_detector_create(tf);
using namespace apriltag_umich;

int main(int argc, char** argv) {
  const std::string package_name = "apriltag_mit";
  const std::string package_path(ros::package::getPath(package_name));
  const std::string image_path(package_path + "/image/tag_sampler.png");
  std::cout << "Image: " << image_path << std::endl;

  // Load image
  cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  cv::imshow("image", image);

  // Process image
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);

  Imageu8Ptr im(image_u8_create_from_gray(image_gray.cols, image_gray.rows,
                                          image_gray.data));
  std::vector<TagFamilyPtr> tag_family;
  tag_family.emplace_back(TagFamilyPtr(tag36h11_create()));
  tag_family.emplace_back(TagFamilyPtr(tag25h9_create()));
  tag_family.emplace_back(TagFamilyPtr(tag16h5_create()));

  for (const TagFamilyPtr& tf : tag_family) {
    TagDetectorPtr td(apriltag_detector_create());
    apriltag_detector_add_family(td.get(), tf.get());
    ZarrayPtr detections(apriltag_detector_detect(td.get(), im.get()));
    const auto num_detections = zarray_size(detections.get());
    std::cout << "Number of detections: " << num_detections << std::endl;
    for (int i = 0; i < num_detections; ++i) {
      apriltag_detection_t* det;
      zarray_get(detections.get(), i, &det);
      cv::line(image, cv::Point2f(det->p[0][0], det->p[0][1]),
               cv::Point2f(det->p[1][0], det->p[1][1]),
               cv::Scalar(255, 0, 0, 0));
      cv::line(image, cv::Point2f(det->p[1][0], det->p[1][1]),
               cv::Point2f(det->p[2][0], det->p[2][1]),
               cv::Scalar(0, 255, 0, 0));
      cv::line(image, cv::Point2f(det->p[2][0], det->p[2][1]),
               cv::Point2f(det->p[3][0], det->p[3][1]),
               cv::Scalar(0, 0, 255, 0));
      cv::line(image, cv::Point2f(det->p[3][0], det->p[3][1]),
               cv::Point2f(det->p[0][0], det->p[0][1]),
               cv::Scalar(255, 0, 255, 0));
      cv::putText(image, std::to_string(det->id),
                  cv::Point2f(det->c[0] - 5, det->c[1] + 5),
                  cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2);
      apriltag_detection_destroy(det);
    }
  }

  cv::imshow("detection", image);
  cv::waitKey(-1);
}
