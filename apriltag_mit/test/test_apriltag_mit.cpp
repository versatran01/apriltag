#include "apriltag_mit/apriltag_mit.h"

#include <iostream>
#include <ros/package.h>
#include <gtest/gtest.h>

using namespace apriltag_mit;

class SampleImageTest : public testing::Test {
 protected:
  SampleImageTest()
      : package_name_("apriltag_mit"),
        package_path_(ros::package::getPath(package_name_)),
        image_path_(package_path_ + "/image/tag_sampler.png"),
        test_image_(cv::imread(image_path_, CV_LOAD_IMAGE_GRAYSCALE)) {}

  std::string package_name_, package_path_, image_path_;
  cv::Mat test_image_;
};

TEST_F(SampleImageTest, Tag36h11) {
  TagDetector tag_detector(tagCodes36h11);
  std::vector<TagDetection> tag_detection =
      tag_detector.extractTags(test_image_);
  ASSERT_EQ(4, tag_detection.size());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// Old stuff
// int main(int argc, char** argv) {
//  const std::string package_name = "apriltag_mit";
//  const std::string package_path(ros::package::getPath(package_name));
//  const std::string image_path(package_path + "/image/tag_sampler.png");
//  std::cout << "Image: " << image_path << std::endl;

//  // Load image
//  cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
//  cv::imshow("image", image);

//  // Process image
//  cv::Mat image_gray;
//  cv::cvtColor(image, image_gray, CV_BGR2GRAY);

//  // There are three tag families in this image
//  std::vector<TagCodes> all_tag_codes = {tagCodes36h11, tagCodes25h9,
//                                         tagCodes16h5};

//  for (const TagCodes& tag_codes : all_tag_codes) {
//    TagDetector tag_detector(tag_codes);
//    std::vector<TagDetection> tag_detection =
//        tag_detector.extractTags(image_gray);
//    std::cout << "Number of detections: " << tag_detection.size() <<
//    std::endl;
//    for (const TagDetection& tag : tag_detection) {
//      std::cout << tag.p[0].first << " " << tag.p[0].second << std::endl;
//      tag.draw(image);
//    }
//  }

//  cv::imshow("detection", image);
//  cv::waitKey(-1);
//}
