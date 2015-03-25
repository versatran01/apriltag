#ifndef APRILTAG_RVIZ_APRILTAG_VISUAL_H_
#define APRILTAG_RVIZ_APRILTAG_VISUAL_H_

#include <OGRE/OgreSceneManager.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>

namespace apriltag_rviz {

class ApriltagVisual {
 public:
  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* parent_node);

  virtual ~ApriltagVisual() = default;

  void setMessage(const apriltag_msgs::ApriltagArrayStampedConstPtr& msg);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColor(float r, float g, float b, float a);

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  boost::shared_ptr<rviz::Arrow> arrow_;
  boost::shared_ptr<rviz::Axes> axes_;
};

}  // namespace apriltag_rviz

#endif  // APRILTAG_RVIZ_APRILTAG_VISUAL_H_
