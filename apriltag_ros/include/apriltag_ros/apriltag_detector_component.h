#ifndef APRILTAG_ROS_APRILTAG_DETECTOR_COMPONENT_H_
#define APRILTAG_ROS_APRILTAG_DETECTOR_COMPONENT_H_

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <apriltag_msgs/msg/apriltag_array_stamped.hpp>
#include "apriltag_ros/apriltag_detector.h"

namespace apriltag_ros {

namespace it = image_transport;

class ApriltagDetectorComponent : public rclcpp::Node {
 public:
  using ApriltagArrayStamped = apriltag_msgs::msg::ApriltagArrayStamped;
  typedef sensor_msgs::msg::Image Image;
  typedef std::shared_ptr<const Image> ImageConstPtr;
  explicit ApriltagDetectorComponent(const rclcpp::NodeOptions &options);
  ~ApriltagDetectorComponent();

 private:
  void imageCb(const Image::ConstSharedPtr &image_msg);
  rcl_interfaces::msg::SetParametersResult parametersCb(
      const std::vector<rclcpp::Parameter> &parameters);
  it::Subscriber sub_image_;  // subscribe to camera image
  it::Publisher pub_disp_;    // publish debug image
  rclcpp::Publisher<ApriltagArrayStamped>::SharedPtr pub_tags_;
  ApriltagDetectorPtr detector_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      paramCallbackHandle_;
};

}  // namespace apriltag_ros

#endif // APRILTAG_ROS_APRILTAG_DETECTOR_NODE_H_
