#include "apriltag_ros/apriltag_detector.h"
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>

using namespace apriltag_ros;

int main() {
  const std::string package_name("apriltag_ros");
  const std::string package_path(ros::package::getPath(package_name));
  const std::string image_path("/home/chao/Downloads/test.png");
  cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

  auto detector =
      ApriltagDetector::Create(DetectorType::Mit, TagFamily::tf36h11);
  detector->Detect(image);
  detector->Draw(image);
  cv::namedWindow("image", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
  cv::imshow("image", image);
  cv::waitKey(-1);
}
