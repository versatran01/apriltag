#include <iostream>
#include <chrono>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag16h5.h"

#include <ros/package.h>

using namespace AprilTags;
using namespace std::chrono;

int main(int argc, char** argv) {
  high_resolution_clock::time_point start;
  milliseconds elapsed;

  const std::string package_name = "apriltag_mit";
  const std::string package_path(ros::package::getPath(package_name));
  const std::string image_path(package_path + "/image/tag_sampler.png");
  std::cout << "Image: " << image_path << std::endl;

  // Load image
  cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  cv::imshow("original", image);

  // Process image
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);

  cv::Mat image_copy;
  image.copyTo(image_copy);

  start = std::chrono::high_resolution_clock::now();
  cv::Mat image_half;
  cv::resize(image_gray, image_half, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
  elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
  std::cout << "Resize image: " << elapsed.count() << std::endl;
  cv::imshow("half", image_half);

  // There are three tag families in this image
  std::vector<TagCodes> all_tag_codes = {tagCodes36h11, tagCodes25h9,
                                         tagCodes16h5};

  for (const TagCodes& tag_codes : all_tag_codes) {
    TagDetector tag_detector(tag_codes);

    start = std::chrono::high_resolution_clock::now();
    std::vector<TagDetection> orig_detection =
        tag_detector.extractTags(image_gray);
    elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Detection in orig image: " << elapsed.count() << std::endl;

    start = std::chrono::high_resolution_clock::now();
    std::vector<TagDetection> half_detection =
        tag_detector.extractTags(image_half);
    elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Detection in half image: " << elapsed.count() << std::endl;

    for (size_t i = 0; i < orig_detection.size(); ++i) {
      const TagDetection& orig_tag = orig_detection[i];
      const TagDetection& half_tag = half_detection[i];
      orig_tag.draw(image);

      cv::Point2f orig_p0(orig_tag.p[0].first, orig_tag.p[0].second);
      cv::Point2f half_p0(half_tag.p[0].first * 2, half_tag.p[1].second * 2);

      std::vector<cv::Point2f> corners;
      for (const auto& c : half_tag.p) {
        corners.push_back(cv::Point2f(c.first * 2, c.second * 2));
      }
      cv::Size win_size = cv::Size(5, 5);
      cv::Size zero_zone = cv::Size(-1, -1);
      cv::TermCriteria criteria =
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
      start = std::chrono::high_resolution_clock::now();
      cv::cornerSubPix(image_gray, corners, win_size, zero_zone, criteria);
      microseconds elapsed =
          duration_cast<microseconds>(high_resolution_clock::now() - start);
      std::cout << "Corner sub: " << elapsed.count() << std::endl;
      half_p0 = corners.front();

      std::cout << "orig p0: " << orig_p0 << std::endl;
      std::cout << "half p0: " << half_p0 << std::endl;
    }
  }

  cv::imshow("detected", image);
  cv::waitKey(-1);
}
