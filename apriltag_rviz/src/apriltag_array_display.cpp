#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>

#include "apriltag_rviz/apriltag_array_display.h"
#include "apriltag_rviz/apriltag_visual.h"

#include <QtGui/QColor>

namespace apriltag_rviz {

inline bool validateFloats(const apriltag_msgs::Apriltag& msg) {
  return rviz::validateFloats(msg.pose) && rviz::validateFloats(msg.corners) &&
         rviz::validateFloats(msg.corners);
}

inline bool validateFloats(const apriltag_msgs::ApriltagArrayStamped& msg) {
  return rviz::validateFloats(msg.apriltags);
}

ApriltagArrayDisplay::ApriltagArrayDisplay() {
  // Shape property
  shape_property_ =
      new rviz::EnumProperty("Shape", "Arrow", "Shape to display the tags as.",
                             this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", Shape::ARROW);
  shape_property_->addOption("Axes", Shape::AXES);

  // Texture property
  texture_property_ =
      new rviz::EnumProperty("Texture", "Color", "Texture of the tag.", this,
                             SLOT(updateTextureChoice()));
  texture_property_->addOption("Color", Texture::UNIFORM);
  texture_property_->addOption("Tag", Texture::TAG);

  // Display property
  display_property_ =
      new rviz::EnumProperty("Display", "Shape", "Display type of the tag.",
                             this, SLOT(updateDisplayChoice()));
  display_property_->addOption("Shape", Display::SHAPE_ONLY);
  display_property_->addOption("Texture", Display::TEXTURE_ONLY);
  display_property_->addOption("Shape and texture", Display::SHAPE_AND_TEXTURE);

  // Color and alpha property
  color_property_ = new rviz::ColorProperty(
      "Color", QColor(255, 25, 0), "Color to draw the apriltag arrows.", this,
      SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
  alpha_property_->setMax(0.0);
  alpha_property_->setMax(1.0);
}

void ApriltagArrayDisplay::onInitialize() {
  ROS_INFO("ApriltagArrayDisplay onInitialize");

  MFDClass::onInitialize();
}

void ApriltagArrayDisplay::onEnable() {
  MFDClass::onEnable();
  updateShapeVisibility();
}

void ApriltagArrayDisplay::reset() {
  MFDClass::reset();
  apriltag_visuals_.clear();
}

void ApriltagArrayDisplay::updateDisplayChoice() {
  int display_option = display_property_->getOptionInt();
  if (display_option == Display::SHAPE_ONLY) {
    // Show shape, hide texture, show color and alpha if shape is arrow
    shape_property_->setHidden(false);
    texture_property_->setHidden(true);
    hideColorAndAlpha(!useArrow());
  } else if (display_option == Display::TEXTURE_ONLY) {
    // Hide shape, show texture, show alpha, hide color if texture is tag
    shape_property_->setHidden(true);
    texture_property_->setHidden(false);
    alpha_property_->setHidden(false);
    // Hide color if we choose to use tag as texture
    bool use_tag = (texture_property_->getOptionInt() == Texture::TAG);
    color_property_->setHidden(use_tag);
  } else {
    // Show all
    shape_property_->setHidden(false);
    texture_property_->setHidden(false);
    alpha_property_->setHidden(false);
    color_property_->setHidden(false);
  }
}

void ApriltagArrayDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  // TODO: a for loop that update color of each visuals
}

bool ApriltagArrayDisplay::useArrow() const {
  return shape_property_->getOptionInt() == ARROW;
}

void ApriltagArrayDisplay::hideColorAndAlpha(bool use_arrow) {
  color_property_->setHidden(use_arrow);
  alpha_property_->setHidden(use_arrow);
}

void ApriltagArrayDisplay::updateShapeChoice() {
  bool use_arrow = useArrow();

  color_property_->setHidden(!use_arrow);
  alpha_property_->setHidden(!use_arrow);

  updateShapeVisibility();

  context_->queueRender();
}

void ApriltagArrayDisplay::updateShapeVisibility() {
  if (!saw_tags_) {
    // TODO: do something to hide all both arrow and axes
  } else {
    bool use_arrow = useArrow();
    // TODO: do something to see arrow or axes
  }
}

void ApriltagArrayDisplay::processMessage(
    const apriltag_msgs::ApriltagArrayStampedConstPtr& msg) {
  if (!validateFloats(*msg)) {
    setStatus(StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  /// Here we call the rviz::FrameManager to get the transform from the fixed
  /// frame to the frame in the header of this ApriltagArray message.  If it
  /// fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }
}

void ApriltagArrayDisplay::reset() {
  MFDClass::reset();
  saw_tags_ = false;
  updateShapeVisibility();
}

}  // namespace apriltag_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(apriltag_rviz::ApriltagArrayDisplay, rviz::Display)
