#include "apriltag_ros/apriltag_detector.h"
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

using testing::Test;
using testing::Values;
using testing::WithParamInterface;
using namespace apriltag_ros;

class SampleImageTest : public Test {
protected:
  SampleImageTest()
      : package_name_("apriltag_mit"),
        package_path_(ros::package::getPath(package_name_)),
        image_path_(package_path_ + "/image/tag_sampler.png"),
        test_image_(cv::imread(image_path_, CV_LOAD_IMAGE_GRAYSCALE)) {}

  std::string package_name_, package_path_, image_path_;
  cv::Mat test_image_;
};

class ApriltagDetectorMitTest : public SampleImageTest,
                                public WithParamInterface<std::string> {
public:
  ApriltagDetectorMitTest()
      : tag_detector_(ApriltagDetector::Create(DetectorType::Mit, GetParam())) {
  }

protected:
  ApriltagDetectorPtr tag_detector_;
};

class ApriltagDetectorUmichTest : public SampleImageTest,
                                  public WithParamInterface<std::string> {
public:
  ApriltagDetectorUmichTest()
      : tag_detector_(
            ApriltagDetector::Create(DetectorType::Umich, GetParam())) {}

protected:
  ApriltagDetectorPtr tag_detector_;
};

TEST_P(ApriltagDetectorMitTest, Detection) {
  const auto tags = tag_detector_->Detect(test_image_);
  EXPECT_EQ(4, tags.size());
}

INSTANTIATE_TEST_CASE_P(ThreeTagFamilies, ApriltagDetectorMitTest,
                        Values(TagFamily::tf16h5, TagFamily::tf25h9,
                               TagFamily::tf36h11));

TEST_P(ApriltagDetectorUmichTest, Detection) {
  const auto tags = tag_detector_->Detect(test_image_);
  EXPECT_EQ(4, tags.size());
}

INSTANTIATE_TEST_CASE_P(ThreeTagFamilies, ApriltagDetectorUmichTest,
                        Values(TagFamily::tf36h11, TagFamily::tf25h9));

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
