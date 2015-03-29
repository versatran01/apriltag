#include "apriltag_rviz/apriltag_visual.h"

namespace apriltag_rviz {

using namespace apriltag_msgs;

/// ==============
/// ApriltagVisual
/// ==============
ApriltagVisual::ApriltagVisual(Ogre::SceneManager* scene_manager,
                               Ogre::SceneNode* camera_node,
                               ApriltagVisualManager* apriltag_visual_manager)
    : scene_manager_(scene_manager),
      tag_node_(camera_node->createChildSceneNode()),
      arrow_(new rviz::Arrow(scene_manager, tag_node_)),
      axes_(new rviz::Axes(scene_manager, tag_node_)),
      quad_object_(scene_manager->createManualObject()),
      border_object_(scene_manager->createManualObject()),
      visual_manager_(apriltag_visual_manager) {}

ApriltagVisual::ApriltagVisual(Ogre::SceneManager* scene_manager,
                               Ogre::SceneNode* camera_node,
                               ApriltagVisualManager* apriltag_visual_manager,
                               const apriltag_msgs::Apriltag& msg)
    : ApriltagVisual(scene_manager, camera_node, apriltag_visual_manager) {
  id_ = msg.id;
  setMessage(msg);
  updateColorAndAlpha();
  updateShapeVisibility();
  updateTextureVisibility();
}

ApriltagVisual::~ApriltagVisual() {
  scene_manager_->destroyManualObject(quad_object_);
  scene_manager_->destroySceneNode(tag_node_);
}

void ApriltagVisual::setMessage(const Apriltag& msg) {
  setTagPose(msg.pose);
  setShapeGeometry(msg.size);
  setTextureGeometry(msg.size);
}

void ApriltagVisual::setShapeGeometry(float tag_size) {
  int tag_bit = 6;
  axes_->set(tag_size, tag_size / (tag_bit + 2) / 2);
  arrow_->set(tag_size, tag_size / (tag_bit + 2), tag_size / 4, tag_size / 5);
}

void ApriltagVisual::setTextureGeometry(float tag_size) {
  const Ogre::ColourValue& color = visual_manager_->color();
  const float s = tag_size / 2;
  const std::vector<Ogre::Vector3> corners = {
      {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}};
  border_object_->begin("BaseWhiteNoLighting",
                        Ogre::RenderOperation::OT_LINE_STRIP);
  border_object_->colour(color);
  for (const auto& corner : corners) {
    border_object_->position(corner);
  }
  border_object_->index(0);
  border_object_->index(1);
  border_object_->index(2);
  border_object_->index(3);
  border_object_->index(0);
  border_object_->end();
  tag_node_->attachObject(border_object_);

  quad_object_->begin(visual_manager_->uniform_material()->getName(),
                      Ogre::RenderOperation::OT_TRIANGLE_LIST);
  quad_object_->colour(color);
  for (const auto& corner : corners) {
    quad_object_->position(corner);
  }

  quad_object_->quad(0, 1, 2, 3);
  quad_object_->quad(3, 2, 1, 0);

  quad_object_->end();
  tag_node_->attachObject(quad_object_);
}

void ApriltagVisual::setTagPose(const geometry_msgs::Pose& pose) {
  const Ogre::Vector3 position(pose.position.x, pose.position.y,
                               pose.position.z);
  const Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x,
                                     pose.orientation.y, pose.orientation.z);
  setTagPose(position, orientation);
}

void ApriltagVisual::setTagPose(const Ogre::Vector3& position,
                                const Ogre::Quaternion& orientation) {
  setTagPosition(position);
  setTagOrientation(orientation);
}

void ApriltagVisual::setTagPosition(const Ogre::Vector3& position) {
  tag_node_->setPosition(position);
}

void ApriltagVisual::setTagOrientation(const Ogre::Quaternion& orientation) {
  tag_node_->setOrientation(orientation);
}

void ApriltagVisual::updateColorAndAlpha() {
  arrow_->setColor(visual_manager_->color());
}

void ApriltagVisual::updateShapeVisibility() {
  arrow_->getSceneNode()->setVisible(visual_manager_->show_shape &&
                                     !visual_manager_->use_axes);
  axes_->getSceneNode()->setVisible(visual_manager_->show_shape &&
                                    visual_manager_->use_axes);
}

void ApriltagVisual::updateTextureVisibility() {
  quad_object_->setVisible(visual_manager_->show_texture &&
                           visual_manager_->use_uniform);
  border_object_->setVisible(visual_manager_->show_texture &&
                             visual_manager_->use_uniform);
}

}  // namespace apriltag_rviz
