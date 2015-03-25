#include "apriltag_rviz/apriltag_visual.h"

namespace apriltag_rviz {

using namespace apriltag_msgs;

ApriltagVisual::ApriltagVisual(Ogre::SceneManager* scene_manager,
                               Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager),
      frame_node_(parent_node->createChildSceneNode()) {}

ApriltagVisual::~ApriltagVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void ApriltagVisual::setMessage(const ApriltagArrayStampedConstPtr& msg) {
  // stuff
}

void ApriltagVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void ApriltagVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void ApriltagVisual::setColor(float r, float g, float b, float a) {
  // stuff
}

}  // namespace apriltag_rviz
