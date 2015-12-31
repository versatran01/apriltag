#ifndef APRILTAGS_LINESEGMENT2D_H_
#define APRILTAGS_LINESEGMENT2D_H_

#include <cmath>

#include "AprilTags/Line2D.h"
#include "AprilTags/XYW.h"

namespace AprilTags {

//! A 2D line with endpoints.
class LineSegment2D {
 public:
  LineSegment2D(const cv::Point2f &p0, const cv::Point2f &p1);
  static LineSegment2D LsqFitXyw(const std::vector<XYW> &xyws);

  const cv::Point2f &p0() const { return p0_; }
  const cv::Point2f &p1() const { return p1_; }

 private:
  Line2D line_;
  cv::Point2f p0_, p1_;
};

}  // namespace

#endif  // APRILTAGS_GLINESEGMENT2D_H_
