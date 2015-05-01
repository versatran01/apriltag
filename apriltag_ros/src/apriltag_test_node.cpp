#include "apriltag_ros/apriltag_test_node.h"

namespace apriltag_ros {

ApriltagTestNode::ApriltagTestNode(const ros::NodeHandle& pnh) : pnh_(pnh) {
  if (!pnh_.getParam("bag_path", bag_path_)) {
    throw std::runtime_error("No param bag_path.");
  }
  pnh_.param("bag_start", bag_start_, 0.0);
  auto sync_cb = boost::bind(&ApriltagTestNode::cameraCb, this, _1, _2);
  exact_sync_ =
      boost::make_shared<ExactSync>(ExactPolicy(queue_size), sub_l_image,
                                    sub_r_image, sub_l_cinfo, sub_r_cinfo);
}

void ApriltagTestNode::process() {
  rosbag::Bag bag(bag_path_);
  const auto stereo_query = rosbag::TopicQuery(stereo_topic_.topics);
  rosbag::View full_view(bag, stereo_query);
  const auto begin_time = full_view.getBeginTime();
  rosbag::View view(bag, stereo_query, begin_time + ros::Duration(bag_start_));
  ROS_INFO("Begin time: %fs", (view.getBeginTime() - begin_time).toSec());

  for (auto it = view.begin(), it_end = view.end(); it != it_end; ++it) {
    const rosbag::MessageInstance& m = *it;

    if (m.getTopic() == stereo_topic_.l_image()) {
      const auto image_msg = m.instantiate<Image>();
      if (image_msg) sub_l_image_.newMessage(image_msg);
      continue;
    }

    if (m.getTopic() == stereo_topic_.r_image()) {
      const auto image_msg = m.instantiate<Image>();
      if (image_msg) sub_r_image_.newMessage(image_msg);
      continue;
    }

    if (m.getTopic() == stereo_topic_.l_cinfo()) {
      const auto cinfo_msg = m.instantiate<CameraInfo>();
      if (cinfo_msg) sub_l_cinfo_.newMessage(cinfo_msg);
      continue;
    }

    if (m.getTopic() == stereo_topic_.r_cinfo()) {
      const auto cinfo_msg = m.instantiate<CameraInfo>();
      if (cinfo_msg) sub_r_cinfo_.newMessage(cinfo_msg);
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
    ROS_ERROR("%s: %S", pnh.getNamespace().c_str(), e.what());
  }
}
