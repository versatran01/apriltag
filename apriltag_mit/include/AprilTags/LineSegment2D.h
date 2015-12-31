#ifndef APRILTAGS_LINESEGMENT2D_H_
#define APRILTAGS_LINESEGMENT2D_H_

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <utility>

#include "AprilTags/Line2D.h"
#include "AprilTags/XYWeight.h"

namespace AprilTags {

//! A 2D line with endpoints.
class LineSegment2D {
 public:
  LineSegment2D(const std::pair<float, float> &p0Arg,
                const std::pair<float, float> &p1Arg);
  static LineSegment2D lsqFitXYW(const std::vector<XYW> &xyweight);
  std::pair<float, float> getP0() const { return p0; }
  std::pair<float, float> getP1() const { return p1; }

 private:
  Line2D line;
  std::pair<float, float> p0;
  std::pair<float, float> p1;
  int weight;
};

}  // namespace

#endif  // APRILTAGS_GLINESEGMENT2D_H_
