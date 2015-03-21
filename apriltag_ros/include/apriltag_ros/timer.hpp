#ifndef APRILTAG_ROS_TIMER_HPP_
#define APRILTAG_ROS_TIEMR_HPP_

#include <chrono>
#include <string>
#include <cassert>
#include <ostream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace apriltag_ros {

// Useful typedefs
using sec = std::chrono::seconds;
using ms = std::chrono::milliseconds;
using us = std::chrono::microseconds;
using ns = std::chrono::nanoseconds;

// SIFANE!
template <typename>
struct is_duration : std::false_type {};

template <typename T, typename U>
struct is_duration<std::chrono::duration<T, U>> : std::true_type {};

// Units
template <typename T>
std::string DurationUnit() {
  return "unknown unit";
}

#define TIMER_UNIT(time)             \
  template <>                        \
  std::string DurationUnit<time>() { \
    return #time;                    \
  }

TIMER_UNIT(sec)
TIMER_UNIT(ms)
TIMER_UNIT(us)
TIMER_UNIT(ns)

#undef TIMER_UNIT

template <typename NumT, typename DenT>
double Ratio() {
  typedef typename NumT::period NumPeriod;
  typedef typename DenT::period DenPeriod;
  typedef typename std::ratio_divide<NumPeriod, DenPeriod>::type RatioType;
  return static_cast<double>(RatioType::num) / RatioType::den;
}

namespace bac = boost::accumulators;

/**
 * @brief The Timer class, a minimum timer class
 */
///@todo: Replace high_resolution_clock with ClockType
template <typename DurationT,
          typename ClockT = std::chrono::high_resolution_clock>
class Timer {
  static_assert(is_duration<DurationT>::value, "Not a valid duration type");

 public:
  explicit Timer(const std::string& name, bool start = true) : name_(name) {
    if (start) Start();
  }

  const std::string& name() const { return name_; }

  /**
   * @brief Start, start timer
   */
  void Start() {
    assert(!timing_);
    timing_ = true;
    start_ = std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Stop, stop timer
   */
  void Stop() {
    elapsed_ = std::chrono::duration_cast<DurationT>(
        std::chrono::high_resolution_clock::now() - start_);
    assert(timing_);
    timing_ = false;
    acc_(Elapsed());  // Update accumulator
  }

  /**
   * @brief Sleep
   * @param tick
   */
  template <typename T = DurationT>
  void Sleep(int tick) {
    std::this_thread::sleep_for(T(tick));
  }

  /**
   * @brief Elapsed
   * @return
   */
  template <typename T = DurationT>
  double Elapsed() const {
    return elapsed_.count() * Ratio<DurationT, T>();
  }

  /**
   * @brief ElapsedDuration
   * @return
   */
  template <typename T = DurationT>
  T ElapsedDuration() const {
    return std::chrono::duration_cast<T>(elapsed_);
  }

  /**
   * @brief UnitStr
   * @return
   */
  template <typename T = DurationT>
  std::string UnitStr() {
    return DurationUnit<T>();
  }

  /**
   * @brief BaseUnitStr
   * @return
   */
  std::string BaseUnitStr() { return UnitStr(); }

  /**
   * @brief Mean
   * @return
   */
  template <typename T = DurationT>
  double Mean() const {
    return bac::extract_result<bac::tag::mean>(acc_) * Ratio<DurationT, T>();
  }

  /**
   * @brief Max
   * @return
   */
  template <typename T = DurationT>
  double Max() const {
    return bac::extract_result<bac::tag::max>(acc_) * Ratio<DurationT, T>();
  }

  /**
   * @brief Min
   * @return
   */
  template <typename T = DurationT>
  double Min() const {
    return bac::extract_result<bac::tag::min>(acc_) * Ratio<DurationT, T>();
  }

  /**
   * @brief Sum
   * @return
   */
  template <typename T = DurationT>
  double Sum() const {
    return bac::extract_result<bac::tag::sum>(acc_) * Ratio<DurationT, T>();
  }

 private:
  using AccumulatorFeatures = bac::features<bac::tag::sum, bac::tag::min,
                                            bac::tag::max, bac::tag::mean>;
  std::string name_{"timer"};
  bool timing_{false};
  DurationT elapsed_;
  std::chrono::high_resolution_clock::time_point start_;
  bac::accumulator_set<double, AccumulatorFeatures> acc_;
};

// More useful typedefs
using TimerSec = Timer<sec>;
using TimerMs = Timer<ms>;
using TimerUs = Timer<us>;
using TimerNs = Timer<ns>;

/// Helper function
template <typename ClockType>
void PrintClockData() {
  std::cout << "- precision: ";
  // If time unit is less or equal one millisecond
  typedef typename ClockType::period RatioType;
  if (std::ratio_less_equal<RatioType, std::milli>::value) {
    // Convert to and print as milliseconds
    typedef typename std::ratio_multiply<RatioType, std::kilo>::type TT;
    std::cout << std::fixed << static_cast<double>(TT::num) / TT::den << " ms"
              << std::endl;
  } else {
    // Print as seconds
    std::cout << std::fixed
              << static_cast<double>(RatioType::num) / RatioType::den << " sec"
              << std::endl;
  }
  std::cout << "- is_steady: " << std::boolalpha << ClockType::is_steady
            << std::endl;
}

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS_TIMER_HPP_
