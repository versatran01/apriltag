#include "apriltag_ros/apriltag_detector.h"
#include <opencv2/highgui/highgui.hpp>

using namespace apriltag_ros;
int main(int argc, char** argv) {
  ApriltagDetectorMit detector(apriltag_mit::tagCodes36h11);
  std::cout << detector.type() << std::endl;
}
