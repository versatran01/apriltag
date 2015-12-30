#ifndef APRILTAGS_QUAD_H_
#define APRILTAGS_QUAD_H_

#include <opencv2/core/core.hpp>

#define INTERPOLATE

namespace AprilTags {

class FloatImage;
class Segment;

//! Represents four segments that form a loop, and might be a tag.
class Quad {
 public:
  /**
   * @brief kMinEdgeLength Minimum size of a tag (in pixels) as measured along
   * edges and diagonals
   */
  static const int kMinEdgeLength = 6;
  /**
   * @brief kMaxQuadAspectRatio Early pruning of quads with insane ratios
   */
  static constexpr float kMaxQuadAspectRatio = 32.0;

  Quad(const std::vector<cv::Point2f>& p);

  //! Interpolate given that the lower left corner of the lower left cell is at
  //(-1,-1) and the upper right corner of the upper right cell is at (1,1).
  cv::Point2f interpolate(const cv::Point2f& p) const;

  //! Same as interpolate, except that the coordinates are interpreted between 0
  // and 1, instead of -1 and 1.
  cv::Point2f interpolate01(const cv::Point2f& p) const;

  //! Points for the quad (in pixel coordinates), in counter clockwise order.
  // These points are the intersections of segments.
  std::vector<cv::Point2f> p;

  //! Segments composing this quad
  std::vector<Segment*> segments;

  //! Total length (in pixels) of the actual perimeter observed for the quad.
  /*! This is in contrast to the geometric perimeter, some of which
   *  may not have been directly observed but rather inferred by
   *  intersecting segments. Quads with more observed perimeter are
   *  preferred over others. */
  float obs_perimeter;

  //! Searches through a vector of Segments to form Quads.
  /*  @param quads any discovered quads will be added to this list
   *  @param path  the segments currently part of the search
   *  @param parent the first segment in the quad
   *  @param depth how deep in the search are we?
   */
  static void search(std::vector<Segment*>& path, Segment& parent, int depth,
                     std::vector<Quad>& quads);

 private:
  cv::Point2f p0_, p3_, p01_, p32_;
};

/**
 * @brief CalcHomography
 * @param p
 * @return
 */
cv::Matx33f CalcHomography(const std::vector<cv::Point2f>& p);

}  // namespace AprilTags

#endif  // APRILTAGS_QUAD_H_
