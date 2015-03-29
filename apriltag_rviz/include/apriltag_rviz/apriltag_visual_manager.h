#ifndef APRILATG_RVIZ_APRILTAG_VISUAL_MANAGER_H_
#define APRILATG_RVIZ_APRILTAG_VISUAL_MANAGER_H_

#include <boost/shared_ptr.hpp>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreColourValue.h>

namespace apriltag_rviz {

class ApriltagVisualManager {
 public:
  using Ptr = boost::shared_ptr<ApriltagVisualManager>;

  ApriltagVisualManager();
  ApriltagVisualManager(Ogre::SceneManager* scene_manager);
  ~ApriltagVisualManager() = default;

  bool use_axes{false};
  bool use_uniform{true};
  bool show_shape{true};
  bool show_texture{false};

  void setSceneManager(Ogre::SceneManager* scene_manager) {
    scene_manager_ = scene_manager;
  }
  void setColor(const Ogre::ColourValue& color) { color_ = color; }
  void setColor(float r, float g, float b) {
    color_.r = r;
    color_.g = g;
    color_.b = b;
  }
  void setAlpha(float a) { color_.a = a; }

  // Some ugly hacks here
  const Ogre::ColourValue& color() const { return color_; }
  float r() const { return color_.r; }
  float g() const { return color_.g; }
  float b() const { return color_.b; }
  float a() const { return color_.a; }
  int ri() const { return floatToInt(r()); }
  int gi() const { return floatToInt(g()); }
  int bi() const { return floatToInt(b()); }

  const Ogre::MaterialPtr& uniform_material() const {
    return uniform_material_;
  }

 private:
  void createUniformMaterial();
  void createTagMaterial();
  int floatToInt(float f) const { return static_cast<int>(f * 255); }

  Ogre::SceneManager* scene_manager_{nullptr};
  Ogre::MaterialPtr uniform_material_;
  Ogre::ColourValue color_;
};

using ApriltagVisualManagerPtr = ApriltagVisualManager::Ptr;
}  // namespace apriltag_rviz

#endif  // APRILATG_RVIZ_APRILTAG_VISUAL_MANAGER_H_
