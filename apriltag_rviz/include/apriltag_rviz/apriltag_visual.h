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

  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node);

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
