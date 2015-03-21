#include "apriltag_ros/timer.hpp"
#include <iostream>

using namespace apriltag_ros;

int main() {
  TimerMs timer("timer", false);
  for (int i = 1; i <= 5; ++i) {
    timer.Start();
    timer.Sleep(i);
    timer.Stop();
    std::cout << "Mean:   " << timer.Mean() << std::endl;
    std::cout << "Max:    " << timer.Max() << std::endl;
    std::cout << "Min:    " << timer.Min() << std::endl;
    std::cout << "Sum:    " << timer.Sum() << std::endl;
  }
}
