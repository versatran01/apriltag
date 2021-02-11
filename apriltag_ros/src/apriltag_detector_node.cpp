#include "apriltag_ros/apriltag_detector_component.h"

#include <rclcpp/rclcpp.hpp>

#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<apriltag_ros::ApriltagDetectorComponent>(
      rclcpp::NodeOptions());
  RCLCPP_INFO(node->get_logger(), "apriltag detector started up!");
  // actually run the node
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return (0);
}
