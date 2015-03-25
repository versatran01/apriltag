#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_pub_pose");
  ros::NodeHandle pnh("~");

  ros::Publisher pub_pose(pnh.advertise<geometry_msgs::PoseStamped>("pose", 1));
  ros::Rate rate(10);
  while (ros::ok()) {
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.w = 1;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pub_pose.publish(pose);
    rate.sleep();
  }
}
