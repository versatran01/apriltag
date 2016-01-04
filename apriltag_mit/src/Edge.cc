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

std::vector<Edge> CalcLocalEdges(float theta0, int x, int y,
                                 const FloatImage &im_mag,
                                 const FloatImage &im_theta) {
  std::vector<Edge> edges;
  edges.reserve(4);  // max is 4

  const int width = im_theta.width();
  const int pid = y * width + x;

  int cost = -1;
  // horizontal edge
  cost = Edge::EdgeCost(theta0, im_theta.get(x + 1, y), im_mag.get(x + 1, y));
  if (cost >= 0) {
    edges.push_back({pid, pid + 1, cost});
  }

  // vertical edge
  cost = Edge::EdgeCost(theta0, im_theta.get(x, y + 1), im_mag.get(x, y + 1));
  if (cost >= 0) {
    edges.push_back({pid, pid + width, cost});
  }

  // downward diagonal edge
  cost = Edge::EdgeCost(theta0, im_theta.get(x + 1, y + 1),
                        im_mag.get(x + 1, y + 1));
  if (cost >= 0) {
    edges.push_back({pid, pid + width + 1, cost});
  }

  // updward diagonal edge
  cost = (x == 0) ? -1 : Edge::EdgeCost(theta0, im_theta.get(x - 1, y + 1),
                                        im_mag.get(x - 1, y + 1));
  if (cost >= 0) {
    edges.push_back({pid, pid + width - 1, cost});
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
