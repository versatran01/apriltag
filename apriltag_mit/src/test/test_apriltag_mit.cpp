#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag16h5.h"

#include <ros/package.h>

// AprilTags::TagDetector tag_detector(AprilTags::tagCodes36h11);
using namespace AprilTags;

int main(int argc, char** argv) {
  const std::string package_name = "apriltag_mit";
  const std::string package_path(ros::package::getPath(package_name));
  const std::string image_path(package_path + "/image/tag_sampler.png");

  // Load image
  cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  cv::imshow("original", image);
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);

  // There are three tag families in this image
  std::vector<TagCodes> all_tag_codes = {tagCodes36h11, tagCodes25h9,
                                         tagCodes16h5};

  for (const TagCodes& tag_codes : all_tag_codes) {
    TagDetector tag_detector(tag_codes);
    std::vector<TagDetection> tag_detection =
        tag_detector.extractTags(image_gray);
    std::cout << "Number of detections: " << tag_detection.size() << std::endl;
    for (const TagDetection& tag : tag_detection) {
      tag.draw(image);
    }
  }

  cv::imshow("detected", image);
  cv::waitKey(-1);
}
