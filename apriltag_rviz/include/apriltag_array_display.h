#ifndef APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_
#define APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_

#include <rviz/message_filter_display.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>

namespace apriltag_rviz {

class ApritlagVisual;

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
}

class ApriltagArrayDisplay
    : public rviz::MessageFilterDisplay<apriltag_msgs::ApriltagArrayStamped> {
  Q_OBJECT
 public:
  ApriltagArrayDisplay();
  virtual ~ApriltagArrayDisplay();

 protected:
  virtual void onInitialize();
  virtual void reset();

 private:
  void processMessage(const apriltag_msgs::ApriltagArrayStampedConstPtr& msg);
  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
};

}  // namespace apriltag_rviz

#endif  // APIRLTAG_RVIZ_APRILTAG_ARRAY_DISPLAY_H_
