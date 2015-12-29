#ifndef APRILTAGS_EDGE_H_
#define APRILTAGS_EDGE_H_

#include <vector>

#include "AprilTags/FloatImage.h"

namespace AprilTags {

class FloatImage;
class UnionFindSimple;

using std::min;
using std::max;

//! Represents an edge between adjacent pixels in the image.
/*! The edge is encoded by the indices of the two pixels. Edge cost
 *  is proportional to the difference in local orientations.
 */
class Edge {
 public:
  /**
   * @brief kMinMag minimum intensity gradient for an edge to be recognized
   */
  static float const kMinMag;

  /**
   * @brief kMaxEdgeCost maximum acceptable difference in local orientation
   */
  static float const kMaxEdgeCost;
  static int const kWeightScale;    // was 10000
  static float const kThetaThresh;  //!< theta threshold for merging edges
  static float const kMagThresh;    //!< magnitude threshold for merging edges

  int pixelIdxA;
  int pixelIdxB;
  int cost;

  //! Constructor
  Edge() : pixelIdxA(), pixelIdxB(), cost() {}

  //! Compare edges based on cost
  inline bool operator<(const Edge &other) const { return (cost < other.cost); }

  //! Cost of an edge between two adjacent pixels; -1 if no edge here
  /*! An edge exists between adjacent pixels if the magnitude of the
    intensity gradient at both pixels is above threshold.  The edge
    cost is proportional to the difference in the local orientation at
    the two pixels.  Lower cost is better.  A cost of -1 means there
    is no edge here (intensity gradien fell below threshold).
   */
  static int edgeCost(float theta0, float theta1, float mag1);

  //! Calculates and inserts up to four edges into 'edges', a vector of Edges.
  static void calcEdges(float theta0, int x, int y, const FloatImage &theta,
                        const FloatImage &mag, std::vector<Edge> &edges,
                        size_t &nEdges);

  //! Process edges in order of increasing cost, merging clusters if we can do
  // so without exceeding the thetaThresh.
  static void mergeEdges(std::vector<Edge> &edges, UnionFindSimple &uf,
                         float tmin[], float tmax[], float mmin[],
                         float mmax[]);
};

}  // namespace AprilTags

#endif  // APRILTAGS_EDGE_H_
