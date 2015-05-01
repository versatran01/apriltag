#include "apriltag_ros/apriltag_detector.h"
#include <gtest/gtest.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>

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
      : tag_detector_(ApriltagDetector::create("mit", GetParam())) {}

 protected:
  ApriltagDetectorPtr tag_detector_;
};

class ApriltagDetectorUmichTest : public SampleImageTest,
                                  public WithParamInterface<std::string> {
 public:
  ApriltagDetectorUmichTest()
      : tag_detector_(ApriltagDetector::create("umich", GetParam())) {}

 protected:
  ApriltagDetectorPtr tag_detector_;
};

TEST_P(ApriltagDetectorMitTest, Detection) {
  tag_detector_->detect(test_image_);
  EXPECT_EQ(4, tag_detector_->tag_detections().size());
}

INSTANTIATE_TEST_CASE_P(ThreeTagFamilies, ApriltagDetectorMitTest,
                        Values("t36h11", "t25h9", "t16h5"));

TEST_P(ApriltagDetectorUmichTest, Detection) {
  tag_detector_->detect(test_image_);
  EXPECT_EQ(4, tag_detector_->tag_detections().size());
}

INSTANTIATE_TEST_CASE_P(ThreeTagFamilies, ApriltagDetectorUmichTest,
                        Values("t36h11", "t25h9"));
