#include "AprilTags/LineSegment2D.h"
#include <limits>

namespace AprilTags {

LineSegment2D::LineSegment2D(const std::pair<float, float> &p0Arg,
                             const std::pair<float, float> &p1Arg)
    : line(p0Arg, p1Arg), p0(p0Arg), p1(p1Arg), weight() {}

LineSegment2D LineSegment2D::lsqFitXYW(const std::vector<XYW> &xyweight) {
  Line2D gline = Line2D::lsqFitXYW(xyweight);
  float maxcoord = -std::numeric_limits<float>::infinity();
  float mincoord = std::numeric_limits<float>::infinity();

  //  for (unsigned int i = 0; i < xyweight.size(); i++) {
  //    std::pair<float, float> p(xyweight[i].x, xyweight[i].y);
  //    float coord = gline.getLineCoordinate(p);
  //    maxcoord = std::max(maxcoord, coord);
  //    mincoord = std::min(mincoord, coord);
  //  }

  for (const XYW &w : xyweight) {
    std::pair<float, float> p(w.x, w.y);
    float coord = gline.getLineCoordinate(p);
    maxcoord = std::max(maxcoord, coord);
    mincoord = std::min(mincoord, coord);
  }

  std::pair<float, float> minValue = gline.getPointOfCoordinate(mincoord);
  std::pair<float, float> maxValue = gline.getPointOfCoordinate(maxcoord);
  return LineSegment2D(minValue, maxValue);
}

}  // namespace AprilTags
