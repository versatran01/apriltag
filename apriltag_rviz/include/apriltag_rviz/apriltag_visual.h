#ifndef APRILTAG_RVIZ_APRILTAG_VISUAL_H_
#define APRILTAG_RVIZ_APRILTAG_VISUAL_H_

#include <OGRE/OgreSceneManager.h>
#include <apriltag_msgs/Apriltag.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgrePlane.h>

namespace apriltag_rviz {

/**
 * @brief The ApriltagVisual class handles the display one apriltag
 */
class ApriltagVisual {
 public:
  using Ptr = boost::shared_ptr<ApriltagVisual>;

  /**
   * @brief The Property struct, default property for creating a new visual
   */
  struct Property {
    bool use_axes{false};
    bool use_uniform{true};
    bool show_shape{true};
    bool show_texture{false};
    float color[4] = {1.0, 0.0, 1.0, 1.0};

    void setColor(const Ogre::ColourValue& color);
    void setColor(float r, float g, float b);
    void setAlpha(float a) { color[3] = a; }

    // Some ugly hacks here
    Ogre::ColourValue ogreColor() const {
      return Ogre::ColourValue(color[0], color[1], color[2], color[3]);
    }
    float r() const { return color[0]; }
    float g() const { return color[1]; }
    float b() const { return color[2]; }
    float a() const { return color[3]; }
    int ri() const { return floatToInt(r()); }
    int gi() const { return floatToInt(g()); }
    int bi() const { return floatToInt(b()); }

   private:
    int floatToInt(float f) const { return static_cast<int>(f * 255); }
  };

  static Property property;

  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node);
  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node,
                 const apriltag_msgs::Apriltag& msg);
  virtual ~ApriltagVisual();

  int id() const { return id_; }

  /// setSomething will set the visual based on external input
  void setMessage(const apriltag_msgs::Apriltag& msg);
  void setShapeGeometry(float tag_size);
  void setTextureGeometry(float tag_size);
  void setTagPose(const geometry_msgs::Pose& pose);
  void setTagPose(const Ogre::Vector3& position,
                  const Ogre::Quaternion& orientation);
  void setTagPosition(const Ogre::Vector3& position);
  void setTagOrientation(const Ogre::Quaternion& orientation);

  /// updateSomething will update the visual based on static property
  void updateColorAndAlpha();
  void updateShapeVisibility();
  void updateTextureVisibility();

 private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* tag_node_;
  int id_{0};

  boost::shared_ptr<rviz::Arrow> arrow_;
  boost::shared_ptr<rviz::Axes> axes_;

  Ogre::ManualObject* tag_object_;
  //  Ogre::MaterialPtr uniform_material_;
};

using ApriltagVisualPtr = ApriltagVisual::Ptr;

}  // namespace apriltag_rviz

#endif  // APRILTAG_RVIZ_APRILTAG_VISUAL_H_
