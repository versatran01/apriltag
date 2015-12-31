#ifndef APRILTAGS_EDGE_H_
#define APRILTAGS_EDGE_H_

#include <vector>

#include "AprilTags/FloatImage.h"

namespace AprilTags {

class FloatImage;
class UnionFind;

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
  // float const Edge::minMag = 0.004f;
  static constexpr float kMinMag = 0.06f;

  /**
   * @brief kMaxEdgeCost maximum acceptable difference in local orientation
   */
  static constexpr float kMaxEdgeCost = 30.f * float(M_PI) / 180.f;
  static constexpr int kWeightScale = 100;  // was 10000
  // theta threshold for merging edges
  static constexpr float kThetaThresh = 100;
  // magnitude threshold for merging edges
  static constexpr float kMagThresh = 1200;

  int pid0;
  int pid1;
  int cost;

  //! Constructor
  Edge() : pid0(), pid1(), cost() {}

  //! Compare edges based on cost
  inline bool operator<(const Edge &other) const { return (cost < other.cost); }

  //! Cost of an edge between two adjacent pixels; -1 if no edge here
  /*! An edge exists between adjacent pixels if the magnitude of the
    intensity gradient at both pixels is above threshold.  The edge
    cost is proportional to the difference in the local orientation at
    the two pixels.  Lower cost is better.  A cost of -1 means there
    is no edge here (intensity gradien fell below threshold).
   */
  static int EdgeCost(float theta0, float theta1, float mag1);

  //! Calculates and inserts up to four edges into 'edges', a vector of Edges.
  static void CalcEdges(float theta0, int x, int y, const FloatImage &theta,
                        const FloatImage &mag, std::vector<Edge> &edges,
                        size_t &nEdges);

  //! Process edges in order of increasing cost, merging clusters if we can do
  // so without exceeding the thetaThresh.
  static void MergeEdges(std::vector<Edge> &edges, UnionFind &uf,
                         float tmin[], float tmax[], float mmin[],
                         float mmax[]);
};

}  // namespace AprilTags

#endif  // APRILTAGS_EDGE_H_
