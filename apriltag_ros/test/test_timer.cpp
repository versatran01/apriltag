#include <gtest/gtest.h>
#include "apriltag_ros/timer.hpp"

using namespace apriltag_ros;
using testing::Test;

TEST(TimerUnitTest, AllTimerUnits) {
  TimerMs timer_ms("name", false);
  ASSERT_EQ(std::string("ms"), timer_ms.baseUnitStr());
  TimerUs timer_us("name", false);
  ASSERT_EQ(std::string("us"), timer_us.baseUnitStr());
  TimerNs timer_ns("name", false);
  ASSERT_EQ(std::string("ns"), timer_ns.baseUnitStr());
  TimerSec timer_sec("name", false);
  ASSERT_EQ(std::string("sec"), timer_sec.baseUnitStr());
}

class TimerElapsedTest : public Test {
 protected:
  TimerElapsedTest() : timer_("timer", false), tick_(50) {}

  TimerMs timer_;
  int tick_;
};

TEST_F(TimerElapsedTest, OnConstruction) {
  ASSERT_DOUBLE_EQ(0.0, timer_.elapsed());
}

TEST_F(TimerElapsedTest, SmallDuration) {
  timer_.start();
  timer_.sleep(tick_);
  timer_.stop();
  ASSERT_DOUBLE_EQ(tick_, timer_.elapsed());
}
