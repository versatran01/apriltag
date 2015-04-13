#include "apriltag_rviz/apriltag_visual_manager.h"

namespace apriltag_rviz {

ApriltagVisualManager::ApriltagVisualManager() : color_(1.0, 0.0, 1.0, 1.0) {}

ApriltagVisualManager::ApriltagVisualManager(Ogre::SceneManager* scene_manager)
    : scene_manager_(scene_manager), color_(1.0, 0.0, 1.0, 1.0) {
  createUniformMaterial();
}

void ApriltagVisualManager::createUniformMaterial() {
  const Ogre::String& group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::MaterialManager& material_manager =
      Ogre::MaterialManager::getSingleton();
  uniform_material_ = material_manager.create("unifrom_tag", group);
  uniform_material_->setReceiveShadows(false);
  uniform_material_->getTechnique(0)->setLightingEnabled(false);
  uniform_material_->setDepthWriteEnabled(true);
  uniform_material_->setCullingMode(Ogre::CULL_NONE);
  uniform_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
}

}  // namespace apriltag_rviz
