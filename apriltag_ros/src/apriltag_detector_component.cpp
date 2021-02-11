#include "apriltag_ros/apriltag_detector_component.h"

#include <apriltag_msgs/msg/apriltag_array_stamped.hpp>
#include <boost/thread/lock_guard.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace apriltag_ros {

using namespace sensor_msgs;

ApriltagDetectorComponent::ApriltagDetectorComponent(
    const rclcpp::NodeOptions &options)
    : Node("tag_detector", options) {
  const rmw_qos_profile_t qos = rmw_qos_profile_default;
  pub_disp_ = it::create_publisher(this, "disp", qos);
  pub_tags_ = create_publisher<ApriltagArrayStamped>("tags", 3);
  const rmw_qos_profile_t image_qos = rmw_qos_profile_default;
  sub_image_ =
      it::create_subscription(this, "image",
                              std::bind(&ApriltagDetectorComponent::imageCb,
                                        this, std::placeholders::_1),
                              "raw", image_qos);
  int type = declare_parameter("detector", 0);
  int family = declare_parameter("tag_family", 0);
  int decimate = declare_parameter("decimate", 1);
  int nthreads = declare_parameter("nthreads", 1);
  int black_border_width = declare_parameter("black_border_width", 1);
  RCLCPP_INFO_STREAM(get_logger(), "detector type: (MIT=0, UMICH=1): " << type);
  RCLCPP_INFO_STREAM(get_logger(),
                     "tag family: (0=36h11, 1=25h9, 2=16h5): " << family);
  RCLCPP_INFO_STREAM(get_logger(),
                     "black border width: " << black_border_width);
  if (type != 0 && black_border_width != 1) {
    RCLCPP_ERROR_STREAM(
        get_logger(),
        "black border width !=1 only supported for MIT detector!");
  }
  detector_ = ApriltagDetector::Create(static_cast<DetectorType>(type),
                                       static_cast<TagFamily>(family));
  detector_->set_black_border(black_border_width);
  detector_->set_decimate(decimate);
  detector_->set_nthreads(nthreads);
  paramCallbackHandle_ = add_on_set_parameters_callback(std::bind(
      &ApriltagDetectorComponent::parametersCb, this, std::placeholders::_1));
}

ApriltagDetectorComponent::~ApriltagDetectorComponent() {
  if (paramCallbackHandle_) {
    remove_on_set_parameters_callback(paramCallbackHandle_.get());
  }
}

rcl_interfaces::msg::SetParametersResult
ApriltagDetectorComponent::parametersCb(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  if (!detector_) {
    result.successful = false;
    result.reason = "not initialized yet!";
    return (result);
  }
  result.successful = false;
  result.reason = "parameter_not_found";
  for (const auto &p : parameters) {
    if (p.get_name() == "decimate") {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        RCLCPP_INFO_STREAM(get_logger(), "setting decimate to: " << p.as_int());
        detector_->set_decimate(p.as_int());
        result.reason = "set decimate parameter";
        result.successful = true;
      } else {
        result.reason = "invalid type";
      }
    } else if (p.get_name() == "black_border_width") {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        RCLCPP_INFO_STREAM(get_logger(),
                           "setting black border width to: " << p.as_int());
        detector_->set_black_border(p.as_int());
        result.reason = "set black border";
        result.successful = true;
      } else {
        result.reason = "invalid type";
      }
    }
  }
  return (result);
}

void ApriltagDetectorComponent::imageCb(
    const Image::ConstSharedPtr &image_msg) {
  if (pub_tags_->get_subscription_count() == 0 &&
      pub_disp_.getNumSubscribers() == 0) {
    return;
  }
  const auto gray =
      cv_bridge::toCvShare(image_msg, image_encodings::MONO8)->image;
  // run detector
  auto apriltags = detector_->Detect(gray);

  // publish apriltags
  if (pub_tags_->get_subscription_count() > 0) {
    auto apriltag_array_msg = std::make_shared<ApriltagArrayStamped>();
    apriltag_array_msg->header = image_msg->header;
    apriltag_array_msg->apriltags = apriltags;
    pub_tags_->publish(*apriltag_array_msg);
  }

  // publish detection image
  if (pub_disp_.getNumSubscribers() > 0) {
    cv::Mat disp;
    cv::cvtColor(gray, disp, CV_GRAY2BGR);
    DrawApriltags(disp, apriltags);
    cv_bridge::CvImage cv_img(image_msg->header, image_encodings::BGR8, disp);
    pub_disp_.publish(cv_img.toImageMsg());
  }
}

} // namespace apriltag_ros

RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::ApriltagDetectorComponent)
