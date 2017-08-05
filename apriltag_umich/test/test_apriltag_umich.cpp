#include "apriltag_umich/apriltag_umich.h"

#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

using namespace apriltag_umich;
using ::testing::Test;
using ::testing::WithParamInterface;
using ::testing::Values;

class SampleImageTest : public Test {
protected:
  SampleImageTest()
      : package_name_("apriltag_mit"),
        package_path_(ros::package::getPath(package_name_)),
        image_path_(package_path_ + "/image/tag_sampler.png"),
        test_image_(cv::imread(image_path_, CV_LOAD_IMAGE_GRAYSCALE)),
        test_image_u8_(image_u8_create_from_gray(
            test_image_.cols, test_image_.rows, test_image_.data)) {}

  std::string package_name_, package_path_, image_path_;
  cv::Mat test_image_;
  ImageU8Ptr test_image_u8_;
};

class TagFamilyTest : public SampleImageTest,
                      public WithParamInterface<apriltag_family_t *> {
public:
  TagFamilyTest()
      : tag_family_(GetParam()), tag_detector_(apriltag_detector_create()) {
    apriltag_detector_add_family(tag_detector_.get(), tag_family_.get());
  }

protected:
  TagFamilyPtr tag_family_;
  TagDetectorPtr tag_detector_;
};

TEST_P(TagFamilyTest, Detection) {
  ZarrayPtr detections(
      apriltag_detector_detect(tag_detector_.get(), test_image_u8_.get()));
  EXPECT_EQ(4, zarray_size(detections.get()));
}

INSTANTIATE_TEST_CASE_P(TwoTagFamilies, TagFamilyTest,
                        Values(tag36h11_create(), tag25h9_create()));

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
