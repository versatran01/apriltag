#include "apriltag_ros/apriltag_detector.h"
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

using namespace apriltag_ros;
int main(int argc, char** argv) {
  const std::string package_name("apriltag_ros");
  const std::string package_path(ros::package::getPath(package_name));
  const std::string image_path(package_path + "/image/tag36_11_00000.png");

  auto image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  ApriltagDetectorMit detector("36h11");
  ApriltagVec apriltags = detector.Detect(gray);
  std::cout << "Detection: " << apriltags.size() << std::endl;
  detector.Draw(image);

  cv::namedWindow("image", CV_WINDOW_KEEPRATIO | CV_WINDOW_NORMAL);
  cv::imshow("image", image);
  cv::waitKey(-1);
}
