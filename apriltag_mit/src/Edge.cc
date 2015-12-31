#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/UnionFind.h"

namespace AprilTags {

int Edge::EdgeCost(float theta0, float theta1, float mag1) {
  // mag0 was checked by the main routine so no need to recheck here
  if (mag1 < kMinMag) return -1;

  const float theta_diff = std::abs(Mod2Pi(theta1 - theta0));
  if (theta_diff > kMaxEdgeCost) return -1;

  const float norm_diff = theta_diff / kMaxEdgeCost;
  return static_cast<int>(norm_diff * kWeightScale);
}

void Edge::CalcEdges(float theta0, int x, int y, const FloatImage &im_theta,
                     const FloatImage &mag, std::vector<Edge> &edges,
                     size_t &num_edges) {
  int width = im_theta.width();
  int pid = y * width + x;

  // horizontal edge
  int cost1 = EdgeCost(theta0, im_theta.get(x + 1, y), mag.get(x + 1, y));
  if (cost1 >= 0) {
    edges[num_edges].cost = cost1;
    edges[num_edges].pid0 = pid;
    edges[num_edges].pid1 = y * width + x + 1;
    ++num_edges;
  }

  // vertical edge
  int cost2 = EdgeCost(theta0, im_theta.get(x, y + 1), mag.get(x, y + 1));
  if (cost2 >= 0) {
    edges[num_edges].cost = cost2;
    edges[num_edges].pid0 = pid;
    edges[num_edges].pid1 = (y + 1) * width + x;
    ++num_edges;
  }

  // downward diagonal edge
  int cost3 =
      EdgeCost(theta0, im_theta.get(x + 1, y + 1), mag.get(x + 1, y + 1));
  if (cost3 >= 0) {
    edges[num_edges].cost = cost3;
    edges[num_edges].pid0 = pid;
    edges[num_edges].pid1 = (y + 1) * width + x + 1;
    ++num_edges;
  }

  // updward diagonal edge
  int cost4 = (x == 0) ? -1 : EdgeCost(theta0, im_theta.get(x - 1, y + 1),
                                       mag.get(x - 1, y + 1));
  if (cost4 >= 0) {
    edges[num_edges].cost = cost4;
    edges[num_edges].pid0 = pid;
    edges[num_edges].pid1 = (y + 1) * width + x - 1;
    ++num_edges;
  }
}

void Edge::MergeEdges(std::vector<Edge> &edges, UnionFind &uf, float tmin[],
                      float tmax[], float mmin[], float mmax[]) {
  //  for (size_t i = 0; i < edges.size(); i++) {
  //    int ida = edges[i].pixelIdxA;
  //    int idb = edges[i].pixelIdxB;
  for (Edge &e : edges) {
    int ida = e.pid0;
    int idb = e.pid1;

    ida = uf.GetRepresentative(ida);
    idb = uf.GetRepresentative(idb);

    if (ida == idb) continue;

    int sza = uf.GetSetSize(ida);
    int szb = uf.GetSetSize(idb);

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

    if (tmaxab - tminab > 2 * (float)M_PI)  // corner case that's probably not
                                            // too useful to handle correctly,
                                            // oh well.
      tmaxab = tminab + 2 * (float)M_PI;

    float mminab = std::min(mmin[ida], mmin[idb]);
    float mmaxab = std::max(mmax[ida], mmax[idb]);

    // merge these two clusters?
    float costab = (tmaxab - tminab);
    if (costab <= (std::min(costa, costb) + kThetaThresh / (sza + szb)) &&
        (mmaxab - mminab) <=
            std::min(mmax[ida] - mmin[ida], mmax[idb] - mmin[idb]) +
                kMagThresh / (sza + szb)) {
      int idab = uf.ConnectNodes(ida, idb);

      tmin[idab] = tminab;
      tmax[idab] = tmaxab;

      mmin[idab] = mminab;
      mmax[idab] = mmaxab;
    }
  }
}

}  // namespace AprilTags
