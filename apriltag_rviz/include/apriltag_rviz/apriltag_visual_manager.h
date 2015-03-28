#ifndef APRILATG_RVIZ_APRILTAG_VISUAL_MANAGER_H_
#define APRILATG_RVIZ_APRILTAG_VISUAL_MANAGER_H_

#include <boost/shared_ptr.hpp>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterial.h>

namespace apriltag_rviz {

class ApriltagVisualManager {
 public:
  using Ptr = boost::shared_ptr<ApriltagVisualManager>;

  ApriltagVisualManager(Ogre::SceneManager* scene_manager);
  ~ApriltagVisualManager() = default;

  const Ogre::MaterialPtr& uniform_material() const {
    return uniform_material_;
  }

 private:
  void createUniformMaterial();
  void createTagMaterial();

  Ogre::SceneManager* scene_manager_;
  Ogre::MaterialPtr uniform_material_;
};

using ApriltagVisualManagerPtr = ApriltagVisualManager::Ptr;

}  // namespace apriltag_rviz

#endif  // APRILATG_RVIZ_APRILTAG_VISUAL_MANAGER_H_
