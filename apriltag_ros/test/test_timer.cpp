#include <gtest/gtest.h>
#include "apriltag_ros/timer.hpp"

using namespace apriltag_ros;

TEST(TimerUnitTest, AllTimerUnits) {
  TimerMs timer_ms("name", false);
  ASSERT_EQ(std::string("ms"), timer_ms.BaseUnitStr());
  TimerUs timer_us("name", false);
  ASSERT_EQ(std::string("us"), timer_us.BaseUnitStr());
  TimerNs timer_ns("name", false);
  ASSERT_EQ(std::string("ns"), timer_ns.BaseUnitStr());
  TimerSec timer_sec("name", false);
  ASSERT_EQ(std::string("sec"), timer_sec.BaseUnitStr());
}
