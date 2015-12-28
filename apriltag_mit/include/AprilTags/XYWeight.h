#ifndef APRILTAGS_XYWEIGHT_H_
#define APRILTAGS_XYWEIGHT_H_

namespace AprilTags {

//! Represents a triple holding an x value, y value, and weight value.
struct XYW {
  float x, y, w;

  XYW(float x, float y, float w) : x(x), y(y), w(w) {}
};

}  // namespace AprilTags

#endif  // APRILTAGS_XYWEIGHT_H_
