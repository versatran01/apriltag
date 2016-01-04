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
    int ida = dsets.GetRepresentative(e.pid0);
    int idb = dsets.GetRepresentative(e.pid1);

    if (ida == idb) continue;

    int sza = dsets.GetSetSize(ida);
    int szb = dsets.GetSetSize(idb);

    float tmina = tmin[ida], tmaxa = tmax[ida];
    float tminb = tmin[idb], tmaxb = tmax[idb];

    float costa = (tmaxa - tmina);
    float costb = (tmaxb - tminb);

    // bshift will be a multiple of 2pi that aligns the spans of 'b' with 'a'
    // so that we can properly take the union of them.
    float bshift =
        Mod2Pi((tmina + tmaxa) / 2, (tminb + tmaxb) / 2) - (tminb + tmaxb) / 2;

    float tminab = std::min(tmina, tminb + bshift);
    float tmaxab = std::max(tmaxa, tmaxb + bshift);

    // Corner case probably not too useful to hand correctly
    if (tmaxab - tminab > 2 * Pi<float>()) {
      tmaxab = tminab + 2 * Pi<float>();
    }

    float mminab = std::min(mmin[ida], mmin[idb]);
    float mmaxab = std::max(mmax[ida], mmax[idb]);

    // merge these two clusters?
    float costab = (tmaxab - tminab);
    if (costab <= (std::min(costa, costb) + theta_thresh / (sza + szb)) &&
        (mmaxab - mminab) <=
            std::min(mmax[ida] - mmin[ida], mmax[idb] - mmin[idb]) +
                mag_thresh / (sza + szb)) {
      int idab = dsets.ConnectNodes(ida, idb);

      tmin[idab] = tminab;
      tmax[idab] = tmaxab;

      mmin[idab] = mminab;
      mmax[idab] = mmaxab;
    }
  }
}

}  // namespace AprilTags
