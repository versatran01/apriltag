#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "apriltag_rviz/apriltag_array_display.h"
#include "apriltag_rviz/apriltag_visual.h"

#include <qt4/Qt/qcolor.h>

namespace apriltag_rviz {

ApriltagArrayDisplay::ApriltagArrayDisplay() {
  color_property_ = new rviz::ColorProperty(
      "Color", QColor(204, 51, 204), "Color to draw the acceleration arrows.",
      this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
}

}  // namespace apriltag_rviz
