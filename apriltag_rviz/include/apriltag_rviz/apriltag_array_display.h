#ifndef APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_
#define APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_

#include <rviz/message_filter_display.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/selection/selection_handler.h>

#include "apriltag_rviz/apriltag_visual.h"

namespace apriltag_rviz {

class ApritlagVisual;
class ApriltagArrayDisplaySelectionHandler;

// Forward decleration
// namespace rviz {
// class ColorProperty;
// class FloatProperty;
// class IntProperty;
//}

class ApriltagArrayDisplay
    : public rviz::MessageFilterDisplay<apriltag_msgs::ApriltagArrayStamped> {
  Q_OBJECT
 public:
  enum class Shape { Arrow, Axes };

  ApriltagArrayDisplay() = default;
  virtual ~ApriltagArrayDisplay() = default;

 protected:
  virtual void onInitialize();
  virtual void reset();
  virtual void onEnable();

 private Q_SLOTS:
  void updateColorAndAlpha();
  void updateShapeVisibility();
  void updateShapeChoice();
  void updateAxisGeometry();
  void updateArrowGeometry();
  void updateTextureChoice();

 private:
  void processMessage(const apriltag_msgs::ApriltagArrayStamped::ConstPtr& msg);

  // properties related to axes and arrow are disabled
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::EnumProperty* shape_property_;
  rviz::EnumProperty* texture_property_;

  std::vector<ApriltagVisual> visuals_;
};

}  // namespace apriltag_rviz

#endif  // APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_
