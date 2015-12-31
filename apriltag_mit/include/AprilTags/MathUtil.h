#ifndef APRILTAGS_MATHUTIL_H_
#define APRILTAGS_MATHUTIL_H_

#include <cmath>
#include <cfloat>
#include <cstdlib>
#include <utility>
#include <opencv2/core/core.hpp>

namespace AprilTags {

template <typename T>
constexpr T Pi() {
  static_assert(std::is_floating_point<T>::value,
                "Type must be floating point");
  return static_cast<T>(M_PI);
}

template <typename T>
constexpr T Pi_2() {
  static_assert(std::is_floating_point<T>::value,
                "Type must be floating point");
  return static_cast<T>(M_PI / 2);
}

static inline float Distance2D(const cv::Point2f& p0, const cv::Point2f& p1) {
  const auto dx = p0.x - p1.x;
  const auto dy = p0.y - p1.y;
  return std::sqrt(dx * dx + dy * dy);
}

//! Miscellaneous math utilities and fast exp functions.
class MathUtil {
 public:
  //! Returns the square of a value.
  static inline float square(float x) { return x * x; }

  static inline float Distance2D(const std::pair<float, float>& p0,
                                 const std::pair<float, float>& p1) {
    float dx = p0.first - p1.first;
    float dy = p0.second - p1.second;
    return std::sqrt(dx * dx + dy * dy);
  }

  //! Returns a result in [-Pi, Pi]
  static inline float mod2pi(float vin) {
    const float twopi = 2 * (float)M_PI;
    const float twopi_inv = 1.f / (2.f * (float)M_PI);
    float absv = std::abs(vin);
    float q = absv * twopi_inv + 0.5f;
    int qi = (int)q;
    float r = absv - qi * twopi;
    return (vin < 0) ? -r : r;
  }

  //! Returns a value of v wrapped such that ref and v differ by no more than
  //+/- Pi
  static inline float mod2pi(float ref, float v) {
    return ref + mod2pi(v - ref);
  }
};

}  // namespace AprilTags

#endif  // APRILTAGS_MATHUTIL_H_
