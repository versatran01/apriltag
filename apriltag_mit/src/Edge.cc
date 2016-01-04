#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/DisjointSets.h"

namespace AprilTags {

int Edge::EdgeCost(float theta0, float theta1, float mag1) {
  // mag0 was checked by the main routine so no need to recheck here
  if (mag1 < kMinMag) return -1;

  const float theta_diff = std::abs(Mod2Pi(theta1 - theta0));
  if (theta_diff > kMaxThetaDiff) return -1;

  const float norm_diff = theta_diff / kMaxThetaDiff;
  return norm_diff * kWeightScale;
}

std::vector<Edge> CalcLocalEdges(int x, int y, const FloatImage &im_mag,
                                 const FloatImage &im_theta) {
  std::vector<Edge> edges;
  edges.reserve(4);

  const int width = im_theta.width();
  const int pid0 = y * width + x;
  const float theta0 = im_theta.get(pid0);

  const int pid1s[4] = {pid0 + 1, pid0 + width, pid0 + width + 1,
                        pid0 + width - 1};
  int n = 4;
  // Handle a corner case when pixel is on left border
  if (x == 0) n = 3;

  for (int i = 0; i < n; ++i) {
    const auto pid1 = pid1s[i];
    const auto cost =
        Edge::EdgeCost(theta0, im_theta.get(pid1), im_mag.get(pid1));
    if (cost >= 0) {
      edges.emplace_back(pid0, pid1, cost);
    }
  }

  return edges;
}

void MergeEdges(const std::vector<Edge> &edges, DisjointSets &dsets,
                float tmin[], float tmax[], float mmin[], float mmax[],
                float mag_thresh, float theta_thresh) {
  for (const Edge &e : edges) {
    int id0 = dsets.GetRepresentative(e.pid0);
    int id1 = dsets.GetRepresentative(e.pid1);

    if (id0 == id1) continue;

    int sza = dsets.GetSetSize(id0);
    int szb = dsets.GetSetSize(id1);

    float tmin0 = tmin[id0];
    float tmax0 = tmax[id0];
    float tmin1 = tmin[id1];
    float tmax1 = tmax[id1];

    float cost0 = (tmax0 - tmin0);
    float cost1 = (tmax1 - tmin1);

    // bshift will be a multiple of 2pi that aligns the spans of 'b' with 'a'
    // so that we can properly take the union of them.
    float bshift =
        Mod2Pi((tmin0 + tmax0) / 2, (tmin1 + tmax1) / 2) - (tmin1 + tmax1) / 2;

    float tmin01 = std::min(tmin0, tmin1 + bshift);
    float tmax01 = std::max(tmax0, tmax1 + bshift);

    // Corner case probably not too useful to hand correctly
    if (tmax01 - tmin01 > 2 * Pi<float>()) {
      tmax01 = tmin01 + 2 * Pi<float>();
    }

    float mmin01 = std::min(mmin[id0], mmin[id1]);
    float mmax01 = std::max(mmax[id0], mmax[id1]);

    // merge these two clusters?
    float cost01 = (tmax01 - tmin01);
    if (cost01 <= (std::min(cost0, cost1) + theta_thresh / (sza + szb)) &&
        (mmax01 - mmin01) <=
            std::min(mmax[id0] - mmin[id0], mmax[id1] - mmin[id1]) +
                mag_thresh / (sza + szb)) {
      int id0b = dsets.ConnectNodes(id0, id1);

      tmin[id0b] = tmin01;
      tmax[id0b] = tmax01;

      mmin[id0b] = mmin01;
      mmax[id0b] = mmax01;
    }
  }
}

}  // namespace AprilTags
