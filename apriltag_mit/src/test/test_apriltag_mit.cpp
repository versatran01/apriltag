#include <iostream>

#include <ros/package.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "apriltag_mit/apriltag_mit.h"

using namespace apriltag_mit;

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

  cv::imshow("detection", image);
  cv::waitKey(-1);
}
