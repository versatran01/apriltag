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
  detector_ = ApriltagDetector::create("mit", "t36h11");
}

void ApriltagTestNode::cameraCb(const ImageConstPtr& image_msg,
                                const CameraInfoConstPtr& cinfo_msg) {
  model_.fromCameraInfo(cinfo_msg);
  const auto image_raw = cv_bridge::toCvCopy(image_msg)->image;
  cv::Mat image_rect, image_color;
  model_.rectifyImage(image_raw, image_rect);
  cv::cvtColor(image_rect, image_color, CV_GRAY2BGR);

  detector_->detect(image_rect);

  drawDetection(detector_->tag_detections(), image_color);

  const auto tag_view =
      tagView(detector_->tag_detections(), image_color, 19, 2);

  cv::imshow("color", image_color);
  cv::namedWindow("view", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  if (!tag_view.empty()) cv::imshow("view", tag_view);
  cv::waitKey(-1);
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
    if (!ros::ok()) break;
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

void drawDetection(const std::vector<ApriltagDetection>& detections,
                   cv::Mat& image) {
  if (detections.empty()) return;
  for (const ApriltagDetection& td : detections) {
    for (int i = 0; i < 4; ++i) {
      cv::Point point(td.p[i][0], td.p[i][1]);
      cv::circle(image, point, 1, CV_RGB(255, 0, 0), -1);
    }
  }
}

cv::Mat tagView(const std::vector<ApriltagDetection>& detections,
                const cv::Mat& image, int corner_win_size, int tags_per_row) {
  if (detections.empty() || image.empty()) return cv::Mat();
  int s = corner_win_size;
  // Make sure s is odd
  if (s % 2 == 0) ++s;
  int s2 = s * 2 + 1;
  const int tag_cols = tags_per_row;
  const int tag_rows = (detections.size() - 1) / tags_per_row + 1;
  cv::Mat tag_view =
      cv::Mat::zeros(tag_rows * 2 * s2, tag_cols * 2 * s2, CV_8UC3);

  cv::Mat image_pad;
  cv::copyMakeBorder(image, image_pad, s, s, s, s, cv::BORDER_CONSTANT);
  for (int i = 0; i < detections.size(); ++i) {
    // for each tag
    int r = i / tag_cols;
    int c = i - r * tag_cols;
    const ApriltagDetection& td = detections[i];
    for (int j = 0; j < 4; ++j) {
      // for each corner
      int ix = td.p[j][0];
      int iy = td.p[j][1];
      int x_src = ix;  // -s + s
      int y_src = iy;  // -s + s
      // Make sure in bound
      if (ix < 0 || ix >= image.cols || iy < 0 || iy >= image.rows) continue;
      int x_dst = (c * 2 + j - (j / 2) * 2) * s2;
      int y_dst = (r * 2 + j / 2) * s2;
      // calculate w and x_dst
      cv::Mat src(image_pad(cv::Rect(x_src, y_src, s2, s2)));
      cv::Mat dst(tag_view(cv::Rect(x_dst, y_dst, s2, s2)));
      src.copyTo(dst);
    }
  }

  // Draw line to seperate
  drawGrid(tag_view, tag_rows * 2, tag_cols * 2, s2, CV_RGB(0, 255, 0));
  drawGrid(tag_view, tag_rows, tag_cols, s2 * 2, CV_RGB(255, 0, 0));
  return tag_view;
}

void drawGrid(cv::Mat& image, int rows, int cols, int size,
              const cv::Scalar& color) {
  for (int c = 1; c < cols; ++c) {
    int x = c * size;
    cv::line(image, {x, 0}, {x, image.rows}, color);
  }
  for (int r = 1; r < rows; ++r) {
    int y = r * size;
    cv::line(image, {0, y}, {image.cols, y}, color);
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
