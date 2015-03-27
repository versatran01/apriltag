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

bool validateFloats(const apriltag_msgs::Apriltag& msg) {
  return rviz::validateFloats(msg.pose) && rviz::validateFloats(msg.corners) &&
         rviz::validateFloats(msg.corners);
}

bool validateFloats(const apriltag_msgs::ApriltagArrayStamped& msg) {
  for (const apriltag_msgs::Apriltag& apriltag : msg.apriltags) {
    if (!validateFloats(apriltag)) return false;
  }
  return true;
}

ApriltagArrayDisplay::ApriltagArrayDisplay() {
  // Display property
  std::string default_display;
  if (ApriltagVisual::property.show_shape &&
      ApriltagVisual::property.show_texture) {
    default_display = "Shape and texture";
  } else if (ApriltagVisual::property.show_shape) {
    default_display = "Shape";
  } else if (ApriltagVisual::property.show_texture) {
    default_display = "Texture";
  } else {
    ApriltagVisual::property.show_shape = true;
    default_display = "Shape";
  }

  display_property_ = new rviz::EnumProperty("Display", default_display.c_str(),
                                             "Display type of the tag.", this,
                                             SLOT(updateDisplayChoice()));
  display_property_->addOption("Shape", Display::SHAPE_ONLY);
  display_property_->addOption("Texture", Display::TEXTURE_ONLY);
  display_property_->addOption("Shape and texture", Display::SHAPE_AND_TEXTURE);

  // Shape property
  const std::string default_shape =
      (ApriltagVisual::property.use_axes) ? "Axes" : "Arrow";
  shape_property_ = new rviz::EnumProperty("Shape", default_shape.c_str(),
                                           "Shape to display the tags as.",
                                           this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Axes", Shape::AXES);
  shape_property_->addOption("Arrow", Shape::ARROW);

  // Texture property
  const std::string default_texture =
      (ApriltagVisual::property.use_uniform) ? "Uniform" : "Tag";
  texture_property_ = new rviz::EnumProperty("Texture", default_texture.c_str(),
                                             "Texture of the tag.", this,
                                             SLOT(updateTextureChoice()));
  texture_property_->addOption("Uniform", Texture::UNIFORM);
  texture_property_->addOption("Tag", Texture::TAG);

  // Color and alpha property
  const QColor default_color(ApriltagVisual::property.ri(),
                             ApriltagVisual::property.gi(),
                             ApriltagVisual::property.bi());
  color_property_ = new rviz::ColorProperty(
      "Color", default_color, "Color to draw the apriltag arrows.", this,
      SLOT(updateColorAndAlpha()));

  const auto default_alpha = ApriltagVisual::property.a();
  alpha_property_ = new rviz::FloatProperty(
      "Alpha", default_alpha, "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMax(0.0);
  alpha_property_->setMax(1.0);
}

ApriltagArrayDisplay::~ApriltagArrayDisplay() {
  // TODO: This is not recommended?
  scene_manager_->destroySceneNode(camera_node_);
}

void ApriltagArrayDisplay::onInitialize() {
  ROS_INFO("[ApriltagArrayDisplay] On initialize");
  MFDClass::onInitialize();
  updateDisplayChoice();  // TODO: need this?
  // Attach camera node to scene_node_
  camera_node_ = scene_node_->createChildSceneNode();
}

void ApriltagArrayDisplay::onEnable() {
  ROS_INFO("[ApriltagArrayDisplay] On enable");
  MFDClass::onEnable();
  updateShapeVisibility();  // TODO: need this?
}

void ApriltagArrayDisplay::onDisable() {
  ROS_INFO("[ApriltagArrayDisplay] On disable");
  MFDClass::onDisable();
  apriltag_visuals_.clear();
}

void ApriltagArrayDisplay::reset() {
  ROS_INFO("[ApriltagArrayDisplay] Reset");
  MFDClass::reset();
  apriltag_visuals_.clear();
}

void ApriltagArrayDisplay::updateDisplayChoice() {
  // TODO: can this logic be simplified?
  const int display_option = display_property_->getOptionInt();
  if (display_option == Display::SHAPE_ONLY) {
    ROS_INFO("Shape only");
    // Show shape, hide texture, show color and alpha if shape is arrow
    shape_property_->setHidden(false);
    texture_property_->setHidden(true);
    hideColorAndAlpha(useAxesShape());
    ApriltagVisual::property.show_shape = true;
    ApriltagVisual::property.show_texture = false;
  } else if (display_option == Display::TEXTURE_ONLY) {
    ROS_INFO("Texture only");
    // Hide shape, show texture, show alpha, hide color if texture is tag
    shape_property_->setHidden(true);
    texture_property_->setHidden(false);
    alpha_property_->setHidden(false);
    // Show color if we choose to use uniform texture
    color_property_->setHidden(!useUniformTexture());
    ApriltagVisual::property.show_shape = false;
    ApriltagVisual::property.show_texture = true;
  } else {
    ROS_INFO("Texture and shape");
    // Show all
    shape_property_->setHidden(false);
    texture_property_->setHidden(false);
    alpha_property_->setHidden(false);
    color_property_->setHidden(false);
    ApriltagVisual::property.show_shape = true;
    ApriltagVisual::property.show_texture = true;
  }

  updateDisplayVisibility();

  context_->queueRender();
}

void ApriltagArrayDisplay::updateShapeChoice() {
  ROS_INFO("Update shape choice");
  hideColorAndAlpha(useAxesShape());

  updateShapeVisibility();

  context_->queueRender();
}

void ApriltagArrayDisplay::updateTextureChoice() {
  ROS_INFO("Update texture choice");
  color_property_->setHidden(!useUniformTexture());

  updateTextureVisibility();

  context_->queueRender();
}

void ApriltagArrayDisplay::updateDisplayVisibility() {
  updateShapeVisibility();
  updateTextureVisibility();
}

void ApriltagArrayDisplay::updateColorAndAlpha() {
  const float alpha = alpha_property_->getFloat();
  const Ogre::ColourValue color = color_property_->getOgreColor();
  ApriltagVisual::property.setColor(color);
  ApriltagVisual::property.setAlpha(alpha);
  for (const ApriltagVisualPtr& apriltag_visual : apriltag_visuals_) {
    apriltag_visual->updateColorAndAlpha();
  }
}

void ApriltagArrayDisplay::updateShapeVisibility() {
  ApriltagVisual::property.use_axes = useAxesShape();
  for (const ApriltagVisualPtr& apriltag_visual : apriltag_visuals_) {
    apriltag_visual->updateShapeVisibility();
  }
}

void ApriltagArrayDisplay::updateTextureVisibility() {
  ApriltagVisual::property.use_uniform = useUniformTexture();
  for (const ApriltagVisualPtr& apriltag_visual : apriltag_visuals_) {
    apriltag_visual->updateTextureVisibility();
  }
}

bool ApriltagArrayDisplay::useAxesShape() const {
  return shape_property_->getOptionInt() == Shape::AXES;
}

bool ApriltagArrayDisplay::useUniformTexture() const {
  return texture_property_->getOptionInt() == Texture::TAG;
}

void ApriltagArrayDisplay::hideColorAndAlpha(bool use_axes) {
  color_property_->setHidden(use_axes);
  alpha_property_->setHidden(use_axes);
}

void ApriltagArrayDisplay::processMessage(
    const apriltag_msgs::ApriltagArrayStampedConstPtr& msg) {
  ROS_INFO_THROTTLE(5, "Process message");

  if (!validateFloats(*msg)) {
    setStatus(rviz::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Here we call the rviz::FrameManager to get the transform from the fixed
  // frame to the frame in the header of this ApriltagArray message.  If it
  // fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Move camera node accordingly
  camera_node_->setPosition(position);
  camera_node_->setOrientation(orientation);
  //  ROS_INFO(
  //      "Camera position: (%0.2f, %0.2f, %0.2f), "
  //      "camera orientation: (%0.2f, %0.2f, %0.2f, %0.2f)",
  //      position.x, position.y, position.z, orientation.w, orientation.x,
  //      orientation.y, orientation.z);

  apriltag_visuals_.clear();
  for (const apriltag_msgs::Apriltag& apriltag : msg->apriltags) {
    apriltag_visuals_.emplace_back(boost::make_shared<ApriltagVisual>(
        context_->getSceneManager(), camera_node_, apriltag));
  }
}

}  // namespace apriltag_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(apriltag_rviz::ApriltagArrayDisplay, rviz::Display)
