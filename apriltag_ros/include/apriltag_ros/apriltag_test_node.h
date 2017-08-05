#ifndef APRILTAG_ROS_APRILTAG_TEST_NODE_H_
#define APRILTAG_ROS_APRILTAG_TEST_NODE_H_

#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "apriltag_ros/apriltag_detector.h"

namespace apriltag_ros {

using sensor_msgs::Image;
using sensor_msgs::CameraInfo;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfoConstPtr;
using message_filters::sync_policies::ExactTime;
using message_filters::sync_policies::ApproximateTime;

class Track {
public:
private:
};

class Tracker {
public:
private:
  std::vector<Track> tracks_;
};

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
public:
  void newMessage(const boost::shared_ptr<const M> &msg) {
    this->signalMessage(msg);
  }
};

class ApriltagTestNode {
public:
  using ExactPolicy = ExactTime<Image, CameraInfo>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;

  explicit ApriltagTestNode(const ros::NodeHandle &pnh);

  void process();
  void cameraCb(const ImageConstPtr &image_msg,
                const CameraInfoConstPtr &cinfo_msg);

private:
  ros::NodeHandle pnh_;
  BagSubscriber<Image> sub_image_;
  BagSubscriber<CameraInfo> sub_cinfo_;
  boost::shared_ptr<ExactSync> exact_sync_;
  std::string bag_path_;
  std::string image_topic_, cinfo_topic_;
  double bag_start_;
  image_geometry::PinholeCameraModel model_;
  ApriltagDetectorPtr detector_;
};

void drawDetection(const std::vector<ApriltagDetection> &detections,
                   cv::Mat &image);

cv::Mat tagView(const std::vector<ApriltagDetection> &detections,
                const cv::Mat &image, int corner_win_size,
                int tags_per_row = 4);

void drawGrid(cv::Mat &image, int rows, int cols, int size,
              const cv::Scalar &color);
} // namespace apritlag_ros

#endif // APRILTAG_ROS_APRILTAG_TEST_NODE_H_
