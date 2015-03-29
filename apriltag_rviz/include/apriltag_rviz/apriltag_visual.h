#ifndef APRILTAG_RVIZ_APRILTAG_VISUAL_H_
#define APRILTAG_RVIZ_APRILTAG_VISUAL_H_

#include <OGRE/OgreSceneManager.h>
#include <apriltag_msgs/Apriltag.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgrePlane.h>

#include "apriltag_rviz/apriltag_visual_manager.h"

namespace apriltag_rviz {

/**
 * @brief The ApriltagVisual class handles the display one apriltag
 */
class ApriltagVisual {
 public:
  using Ptr = boost::shared_ptr<ApriltagVisual>;

  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node,
                 ApriltagVisualManager* apriltag_visual_manager);
  ApriltagVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* camera_node,
                 ApriltagVisualManager* apriltag_visual_manager,
                 const apriltag_msgs::Apriltag& msg);
  virtual ~ApriltagVisual();

  int id() const { return id_; }

  /// setSomething will set the visual based on external input
  void setMessage(const apriltag_msgs::Apriltag& msg);
  void setShapeGeometry(float tag_size);
  void setTextureGeometry(float tag_size);
  void setNodePose(const geometry_msgs::Pose& pose);
  void setNodePose(const Ogre::Vector3& position,
                  const Ogre::Quaternion& orientation);
  void setNodePosition(const Ogre::Vector3& position);
  void setNodeOrientation(const Ogre::Quaternion& orientation);

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

  Ogre::ManualObject* quad_object_;
  Ogre::ManualObject* border_object_;
  ApriltagVisualManager* visual_manager_;
};

using ApriltagVisualPtr = ApriltagVisual::Ptr;

}  // namespace apriltag_rviz

#endif  // APRILTAG_RVIZ_APRILTAG_VISUAL_H_
