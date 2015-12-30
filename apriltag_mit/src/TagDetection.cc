#include "opencv2/opencv.hpp"
#include <iterator>
#include "AprilTags/TagDetection.h"
#include "AprilTags/MathUtil.h"

namespace AprilTags {

float TagPerimeter(const std::vector<cv::Point2f> &p) {
  float perimeter = 0;
  for (size_t i = 0; i < 3; ++i) {
    perimeter += Distance2D(p[i], p[i + 1]);
  }
  return perimeter;
}

float TagRadius(const std::vector<cv::Point2f> &p) {
  return TagPerimeter(p) / 8.0f;
}

cv::Point2f TagDetection::interpolate(const cv::Point2f &p) const {
  float z = H(2, 0) * p.x + H(2, 1) * p.y + H(2, 2);
  // prevents returning a pair with -NaN, for which gcc 4.4 flubs isnan
  if (z == 0.0) return {0, 0};

  const float x = (H(0, 0) * p.x + H(0, 1) * p.y + H(0, 2)) / z;
  const float y = (H(1, 0) * p.x + H(1, 1) * p.y + H(1, 2)) / z;
  return cv::Point2f(x, y);
}

bool TagDetection::OverlapsTooMuch(const TagDetection &other) const {
  // Compute a sort of "radius" of the two targets. We'll do this by
  // computing the average length of the edges of the quads
  const auto radius = (TagRadius(p) + TagRadius(other.p)) / 2;

  // distance (in pixels) between two tag centers
  const auto dist = Distance2D(cxy, other.cxy);

  // reject pairs where the distance between centroids is smaller than the
  // "radius" of one of the tags.
  return (dist < radius);
}

void TagDetection::ScaleTag(float scale) {
  cxy *= scale;
  for (cv::Point2f &c : p) {
    c *= scale;
  }
}

}  // namespace AprilTags
