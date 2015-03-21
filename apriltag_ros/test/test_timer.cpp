#include <gtest/gtest.h>
#include "apriltag_ros/timer.hpp"

using namespace apriltag_ros;
using testing::Test;

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

class TimerElapsedTest : public Test {
 protected:
  TimerElapsedTest() : timer_("timer", false), tick_(50) {}

  TimerMs timer_;
  int tick_;
};

TEST_F(TimerElapsedTest, OnConstruction) {
  ASSERT_DOUBLE_EQ(0.0, timer_.Elapsed());
}

TEST_F(TimerElapsedTest, SmallDuration) {
  timer_.Start();
  timer_.Sleep(tick_);
  timer_.Stop();
  ASSERT_DOUBLE_EQ(tick_, timer_.Elapsed());
}
