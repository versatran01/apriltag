#ifndef APRILTAG_RVIZ_APRILTAG_VISUAL_H_
#define APRILTAG_RVIZ_APRILTAG_VISUAL_H_

#include <OGRE/OgreSceneManager.h>
#include <apriltag_msgs/Apriltag.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>

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
    bool use_axes{true};
    bool use_uniform{true};
    bool show_shape{false};
    bool show_texture{false};
    float color[4] = {1.0, 0.0, 0.0, 1.0};

    void setColor(const Ogre::ColourValue& color);
    void setColor(float r, float g, float b);
    void setAlpha(float a) { color[4] = a; }

    float r() const { return color[0]; }
    float g() const { return color[1]; }
    float b() const { return color[2]; }
    float a() const { return color[3]; }
  };

  static Property property;

  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node);
  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node,
                 const apriltag_msgs::Apriltag& msg);

  virtual ~ApriltagVisual();

  void setMessage(const apriltag_msgs::Apriltag& msg);

  void setColor(float r, float g, float b, float a);

 private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* tag_node_;

  boost::shared_ptr<rviz::Arrow> arrow_;
  boost::shared_ptr<rviz::Axes> axes_;
};

using ApriltagVisualPtr = ApriltagVisual::Ptr;


}  // namespace apriltag_rviz

#endif  // APRILTAG_RVIZ_APRILTAG_VISUAL_H_
