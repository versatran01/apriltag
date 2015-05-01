#include "apriltag_ros/apriltag_test_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace apriltag_ros {

ApriltagTestNode::ApriltagTestNode(const ros::NodeHandle& pnh) : pnh_(pnh) {
  if (!pnh_.getParam("bag_path", bag_path_)) {
    throw std::runtime_error("No param bag_path.");
  }
  ROS_INFO("Bagfile: %s", bag_path_.c_str());
  int queue_size;
  pnh_.param("queue_size", queue_size, 5);
  pnh_.param("bag_start", bag_start_, 0.0);
  auto sync_cb = boost::bind(&ApriltagTestNode::cameraCb, this, _1, _2);
  exact_sync_ = boost::make_shared<ExactSync>(ExactPolicy(queue_size),
                                              sub_image_, sub_cinfo_);
  exact_sync_->registerCallback(sync_cb);
  image_topic_ = pnh_.resolveName("image");
  cinfo_topic_ = pnh_.resolveName("camera_info");
  ROS_INFO("Image: %s", image_topic_.c_str());
  ROS_INFO("CameraInfo: %s", cinfo_topic_.c_str());
}

void ApriltagTestNode::cameraCb(const ImageConstPtr& image_msg,
                                const CameraInfoConstPtr& cinfo_msg) {
  model_.fromCameraInfo(cinfo_msg);
  const auto image_raw = cv_bridge::toCvCopy(image_msg)->image;
  cv::Mat image_rect;
  model_.rectifyImage(image_raw, image_rect);
  cv::imshow("rect", image_rect);
  cv::waitKey(1);
}

void ApriltagTestNode::process() {
  rosbag::Bag bag(bag_path_);
  std::vector<std::string> topics = {image_topic_, cinfo_topic_};
  const auto camera_query = rosbag::TopicQuery(topics);
  rosbag::View full_view(bag, camera_query);
  const auto begin_time = full_view.getBeginTime();
  rosbag::View view(bag, camera_query, begin_time + ros::Duration(bag_start_));
  ROS_INFO("Begin time: %fs", (view.getBeginTime() - begin_time).toSec());

  for (auto it = view.begin(), it_end = view.end(); it != it_end; ++it) {
    const rosbag::MessageInstance& m = *it;

    if (m.getTopic() == image_topic_) {
      const auto image_msg = m.instantiate<Image>();
      if (image_msg) sub_image_.newMessage(image_msg);
      continue;
    }

    if (m.getTopic() == cinfo_topic_) {
      const auto cinfo_msg = m.instantiate<CameraInfo>();
      if (cinfo_msg) sub_cinfo_.newMessage(cinfo_msg);
      continue;
    }
  }
}

}  // namespace apritlag_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_test_node");
  ros::NodeHandle pnh("~");

  try {
    apriltag_ros::ApriltagTestNode node(pnh);
    node.process();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
