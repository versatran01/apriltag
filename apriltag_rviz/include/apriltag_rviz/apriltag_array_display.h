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

// Forward decleration, which I don't like
// namespace rviz {
// class ColorProperty;
// class FloatProperty;
// class IntProperty;
//}

class ApriltagArrayDisplay
    : public rviz::MessageFilterDisplay<apriltag_msgs::ApriltagArrayStamped> {
  Q_OBJECT
 public:
  // TODO: wrap these enums in a namespace?
  enum Shape { AXES, ARROW };
  enum Texture { UNIFORM, TAG };
  enum Display { SHAPE_ONLY, TEXTURE_ONLY, SHAPE_AND_TEXTURE };

  ApriltagArrayDisplay();
  virtual ~ApriltagArrayDisplay();

  virtual void onInitialize();
  virtual void reset();

 protected:
  /**
   * @brief Overriden from MessageFilterDisplay to get arrow/axes visibility
   * correct.
   */
  virtual void onEnable(); // TODO: do I need this here?

  /**
   * @brief Overriden from MessageFilterDisplay to clear all visuals when
   * disabled
   */
  virtual void onDisable();

 private Q_SLOTS:
  void updateColorAndAlpha();

  void updateShapeVisibility();
  void updateShapeChoice();

  void updateTextureChoice();
  void updateTextureVisibility();

  void updateDisplayChoice();
  void updateDisplayVisibility();

 private:
  void processMessage(const apriltag_msgs::ApriltagArrayStamped::ConstPtr& msg);

  void clear();
  bool useAxesShape() const;
  bool useUniformTexture() const;
  void hideColorAndAlpha(bool use_axes);

  // properties related to axes and arrow are disabled
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::EnumProperty* shape_property_;
  rviz::EnumProperty* texture_property_;
  rviz::EnumProperty* display_property_;

  std::vector<ApriltagVisualPtr> apriltag_visuals_;
};

/**
 * @brief validateFloats
 */
bool validateFloats(const apriltag_msgs::Apriltag& msg);
bool validateFloats(const apriltag_msgs::ApriltagArrayStamped& msg);

}  // namespace apriltag_rviz

#endif  // APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_
