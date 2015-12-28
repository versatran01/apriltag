#include <algorithm>
#include <cmath>
#include <climits>
#include <map>
#include <vector>

#include <Eigen/Dense>

#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/Gaussian.h"
#include "AprilTags/GrayModel.h"
#include "AprilTags/GLine2D.h"
#include "AprilTags/GLineSegment2D.h"
#include "AprilTags/Gridder.h"
#include "AprilTags/Homography33.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/Quad.h"
#include "AprilTags/Segment.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/UnionFindSimple.h"
#include "AprilTags/XYWeight.h"

#include "AprilTags/TagDetector.h"

namespace AprilTags {

using namespace std;

TagDetector::TagDetector(const TagCodes &tag_codes, int black_border)
    : tag_family_(tag_codes), black_border_(black_border) {}

void TagDetector::set_black_border(int black_border) {
  black_border_ = black_border;
}
int TagDetector::black_border() const { return black_border_; }

std::vector<TagDetection> TagDetector::ExtractTags(const cv::Mat &image) const {
  int width = image.cols;
  int height = image.rows;
  AprilTags::FloatImage im_orig(image);
  std::pair<int, int> optical_center(width / 2, height / 2);

  //================================================================
  // Step one: preprocess image (convert to grayscale) and low pass if necessary

  // This is a copy
  FloatImage fim = im_orig;

  if (sampling_sigma_ > 0) {
    int filtsz = ((int)max(3.0f, 3 * sampling_sigma_)) | 1;
    fim.filterFactoredCentered(filtsz, sampling_sigma_);
  }

  //================================================================
  // Step two: Compute the local gradient. We store the direction and magnitude.
  // This step is quite sensitve to noise, since a few bad theta estimates will
  // break up segments, causing us to miss Quads. It is useful to do a Gaussian
  // low pass on this step even if we don't want it for encoding.

  FloatImage fimSeg;
  if (segment_sigma_ > 0) {
    if (segment_sigma_ == sampling_sigma_) {
      fimSeg = fim;
    } else {
      // blur anew
      int filtsz = ((int)max(3.0f, 3 * segment_sigma_)) | 1;
      fimSeg = im_orig;
      fimSeg.filterFactoredCentered(filtsz, segment_sigma_);
    }
  } else {
    fimSeg = im_orig;
  }

  FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
  FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());

  for (int y = 1; y < fimSeg.getHeight() - 1; y++) {
    for (int x = 1; x < fimSeg.getWidth() - 1; x++) {
      float Ix = fimSeg.get(x + 1, y) - fimSeg.get(x - 1, y);
      float Iy = fimSeg.get(x, y + 1) - fimSeg.get(x, y - 1);

      float mag = Ix * Ix + Iy * Iy;
      float theta = atan2(Iy, Ix);

      fimTheta.set(x, y, theta);
      fimMag.set(x, y, mag);
    }
  }

  //================================================================
  // Step three. Extract edges by grouping pixels with similar
  // thetas together. This is a greedy algorithm: we start with
  // the most similar pixels.  We use 4-connectivity.
  UnionFindSimple uf(fimSeg.getWidth() * fimSeg.getHeight());

  vector<Edge> edges(width * height * 4);
  size_t nEdges = 0;

  // Bounds on the thetas assigned to this group. Note that because
  // theta is periodic, these are defined such that the average
  // value is contained *within* the interval.
  {
    // limit scope of storage
    // Previously all this was on the stack, but this is 1.2MB for 320x240
    // images That's already a problem for OS X (default 512KB thread stack
    // size), could be a problem elsewhere for bigger images... so store on heap

    // do all the memory in one big block, exception safe
    vector<float> storage(width * height * 4);
    float *tmin = &storage[width * height * 0];
    float *tmax = &storage[width * height * 1];
    float *mmin = &storage[width * height * 2];
    float *mmax = &storage[width * height * 3];

    for (int y = 0; y + 1 < height; y++) {
      for (int x = 0; x + 1 < width; x++) {
        float mag0 = fimMag.get(x, y);
        if (mag0 < Edge::minMag) continue;
        mmax[y * width + x] = mag0;
        mmin[y * width + x] = mag0;

        float theta0 = fimTheta.get(x, y);
        tmin[y * width + x] = theta0;
        tmax[y * width + x] = theta0;

        // Calculates then adds edges to 'vector<Edge> edges'
        Edge::calcEdges(theta0, x, y, fimTheta, fimMag, edges, nEdges);

        // XXX Would 8 connectivity help for rotated tags?
        // Probably not much, so long as input filtering hasn't been disabled.
      }
    }

    edges.resize(nEdges);
    std::stable_sort(edges.begin(), edges.end());
    Edge::mergeEdges(edges, uf, tmin, tmax, mmin, mmax);
  }

  //================================================================
  // Step four: Loop over the pixels again, collecting statistics for each
  // cluster.
  // We will soon fit lines (segments) to these points.

  map<int, vector<XYWeight> > clusters;
  for (int y = 0; y + 1 < fimSeg.getHeight(); y++) {
    for (int x = 0; x + 1 < fimSeg.getWidth(); x++) {
      if (uf.getSetSize(y * fimSeg.getWidth() + x) <
          Segment::minimumSegmentSize)
        continue;

      int rep = (int)uf.getRepresentative(y * fimSeg.getWidth() + x);

      map<int, vector<XYWeight> >::iterator it = clusters.find(rep);
      if (it == clusters.end()) {
        clusters[rep] = vector<XYWeight>();
        it = clusters.find(rep);
      }
      vector<XYWeight> &points = it->second;
      points.push_back(XYWeight(x, y, fimMag.get(x, y)));
    }
  }

  //================================================================
  // Step five: Loop over the clusters, fitting lines (which we call Segments).
  std::vector<Segment> segments;  // used in Step six
  std::map<int, std::vector<XYWeight> >::const_iterator clustersItr;
  for (clustersItr = clusters.begin(); clustersItr != clusters.end();
       clustersItr++) {
    std::vector<XYWeight> points = clustersItr->second;
    GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

    // filter short lines
    float length = MathUtil::Distance2D(gseg.getP0(), gseg.getP1());
    if (length < Segment::minimumLineLength) continue;

    Segment seg;
    float dy = gseg.getP1().second - gseg.getP0().second;
    float dx = gseg.getP1().first - gseg.getP0().first;

    float tmpTheta = std::atan2(dy, dx);

    seg.setTheta(tmpTheta);
    seg.setLength(length);

    // We add an extra semantic to segments: the vector
    // p1->p2 will have dark on the left, white on the right.
    // To do this, we'll look at every gradient and each one
    // will vote for which way they think the gradient should
    // go. This is way more retentive than necessary: we
    // could probably sample just one point!

    float flip = 0, noflip = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
      XYWeight xyw = points[i];

      float theta = fimTheta.get((int)xyw.x, (int)xyw.y);
      float mag = fimMag.get((int)xyw.x, (int)xyw.y);

      // err *should* be +M_PI/2 for the correct winding, but if we
      // got the wrong winding, it'll be around -M_PI/2.
      float err = MathUtil::mod2pi(theta - seg.getTheta());

      if (err < 0)
        noflip += mag;
      else
        flip += mag;
    }

    if (flip > noflip) {
      float temp = seg.getTheta() + (float)M_PI;
      seg.setTheta(temp);
    }

    float dot = dx * std::cos(seg.getTheta()) + dy * std::sin(seg.getTheta());
    if (dot > 0) {
      seg.setX0(gseg.getP1().first);
      seg.setY0(gseg.getP1().second);
      seg.setX1(gseg.getP0().first);
      seg.setY1(gseg.getP0().second);
    } else {
      seg.setX0(gseg.getP0().first);
      seg.setY0(gseg.getP0().second);
      seg.setX1(gseg.getP1().first);
      seg.setY1(gseg.getP1().second);
    }

    segments.push_back(seg);
  }

  // Step six: For each segment, find segments that begin where this segment
  // ends. (We will chain segments together next...) The gridder accelerates the
  // search by building (essentially) a 2D hash table.
  Gridder<Segment> gridder(0, 0, width, height, 10);

  // add every segment to the hash table according to the position of the
  // segment's first point. Remember that the first point has a specific meaning
  // due to our left-hand rule above.
  for (unsigned int i = 0; i < segments.size(); i++) {
    gridder.add(segments[i].getX0(), segments[i].getY0(), &segments[i]);
  }

  // Now, find child segments that begin where each parent segment ends.
  for (unsigned i = 0; i < segments.size(); i++) {
    Segment &parentseg = segments[i];

    // compute length of the line segment
    GLine2D parentLine(
        std::pair<float, float>(parentseg.getX0(), parentseg.getY0()),
        std::pair<float, float>(parentseg.getX1(), parentseg.getY1()));

    Gridder<Segment>::iterator iter = gridder.find(
        parentseg.getX1(), parentseg.getY1(), 0.5f * parentseg.getLength());
    while (iter.hasNext()) {
      Segment &child = iter.next();
      if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) {
        continue;
      }

      // compute intersection of points
      GLine2D childLine(std::pair<float, float>(child.getX0(), child.getY0()),
                        std::pair<float, float>(child.getX1(), child.getY1()));

      std::pair<float, float> p = parentLine.intersectionWith(childLine);
      if (p.first == -1) {
        continue;
      }

      float parentDist = MathUtil::Distance2D(
          p, std::pair<float, float>(parentseg.getX1(), parentseg.getY1()));
      float childDist = MathUtil::Distance2D(
          p, std::pair<float, float>(child.getX0(), child.getY0()));

      if (max(parentDist, childDist) > parentseg.getLength()) {
        continue;
      }

      // everything's OK, this child is a reasonable successor.
      parentseg.children.push_back(&child);
    }
  }

  //================================================================
  // Step seven: Search all connected segments to see if any form a loop of
  // length 4.
  // Add those to the quads list.
  vector<Quad> quads;

  vector<Segment *> tmp(5);
  for (unsigned int i = 0; i < segments.size(); i++) {
    tmp[0] = &segments[i];
    Quad::search(im_orig, tmp, segments[i], 0, quads, optical_center);
  }

  //================================================================
  // Step eight. Decode the quads. For each quad, we first estimate a
  // threshold color to decide between 0 and 1. Then, we read off the
  // bits and see if they make sense.

  std::vector<TagDetection> detections;

  for (unsigned int qi = 0; qi < quads.size(); qi++) {
    Quad &quad = quads[qi];

    // Find a threshold
    GrayModel blackModel, whiteModel;
    const int dd = 2 * black_border_ + tag_family_.dimension_bits();

    for (int iy = -1; iy <= dd; iy++) {
      float y = (iy + 0.5f) / dd;
      for (int ix = -1; ix <= dd; ix++) {
        float x = (ix + 0.5f) / dd;
        std::pair<float, float> pxy = quad.interpolate01(x, y);
        int irx = (int)(pxy.first + 0.5);
        int iry = (int)(pxy.second + 0.5);
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) continue;
        float v = fim.get(irx, iry);
        if (iy == -1 || iy == dd || ix == -1 || ix == dd)
          whiteModel.addObservation(x, y, v);
        else if (iy == 0 || iy == (dd - 1) || ix == 0 || ix == (dd - 1))
          blackModel.addObservation(x, y, v);
      }
    }

    bool bad = false;
    code_t tag_code = 0;
    for (int iy = tag_family_.dimension_bits() - 1; iy >= 0; iy--) {
      float y = (black_border_ + iy + 0.5f) / dd;
      for (int ix = 0; ix < tag_family_.dimension_bits(); ix++) {
        float x = (black_border_ + ix + 0.5f) / dd;
        std::pair<float, float> pxy = quad.interpolate01(x, y);
        int irx = (int)(pxy.first + 0.5);
        int iry = (int)(pxy.second + 0.5);
        if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
          bad = true;
          continue;
        }
        float threshold =
            (blackModel.interpolate(x, y) + whiteModel.interpolate(x, y)) *
            0.5f;
        float v = fim.get(irx, iry);
        tag_code = tag_code << 1;
        if (v > threshold) tag_code |= 1;
      }
    }

    if (!bad) {
      auto td = tag_family_.Decode(tag_code);

      // compute the homography (and rotate it appropriately)
      td.H = quad.homography.getH();
      td.hxy = quad.homography.getCXY();

      float c = std::cos(td.num_rotations * (float)M_PI / 2);
      float s = std::sin(td.num_rotations * (float)M_PI / 2);
      Eigen::Matrix3d R;
      R.setZero();
      R(0, 0) = R(1, 1) = c;
      R(0, 1) = -s;
      R(1, 0) = s;
      R(2, 2) = 1;
      Eigen::Matrix3d tmp;
      tmp = td.H * R;
      td.H = tmp;

      // Rotate points in detection according to decoded
      // orientation.  Thus the order of the points in the
      // detection object can be used to determine the
      // orientation of the target.
      std::pair<float, float> bottomLeft = td.interpolate(-1, -1);
      int best_rot = -1;
      float best_dist = FLT_MAX;
      for (int i = 0; i < 4; i++) {
        float const dist =
            AprilTags::MathUtil::Distance2D(bottomLeft, quad.quadPoints[i]);
        if (dist < best_dist) {
          best_dist = dist;
          best_rot = i;
        }
      }

      for (int i = 0; i < 4; i++) {
        const auto p = quad.quadPoints[(i + best_rot) % 4];
        td.p[i] = cv::Point2f(p.first, p.second);
      }

      if (td.good) {
        const auto c = quad.interpolate01(0.5f, 0.5f);
        td.cxy = cv::Point2f(c.first, c.second);
        td.obs_perimeter = quad.observedPerimeter;
        detections.push_back(td);
      }
    }
  }

  //================================================================
  // Step nine: Some quads may be detected more than once, due to
  // partial occlusion and our aggressive attempts to recover from
  // broken lines. When two quads (with the same id) overlap, we will
  // keep the one with the lowest error, and if the error is the same,
  // the one with the greatest observed perimeter.

  std::vector<TagDetection> good_td;

  // NOTE: allow multiple non-overlapping detections of the same target.

  for (vector<TagDetection>::const_iterator it = detections.begin();
       it != detections.end(); it++) {
    const TagDetection &thisTagDetection = *it;

    bool newFeature = true;

    for (unsigned int odidx = 0; odidx < good_td.size(); odidx++) {
      TagDetection &otherTagDetection = good_td[odidx];

      if (thisTagDetection.id != otherTagDetection.id ||
          !thisTagDetection.OverlapsTooMuch(otherTagDetection))
        continue;

      // There's a conflict.  We must pick one to keep.
      newFeature = false;

      // This detection is worse than the previous one... just don't use it.
      if (thisTagDetection.hamming_distance >
          otherTagDetection.hamming_distance)
        continue;

      // Otherwise, keep the new one if it either has strictly *lower* error, or
      // greater perimeter.
      if (thisTagDetection.hamming_distance <
              otherTagDetection.hamming_distance ||
          thisTagDetection.obs_perimeter > otherTagDetection.obs_perimeter)
        good_td[odidx] = thisTagDetection;
    }

    if (newFeature) good_td.push_back(thisTagDetection);
  }

  return good_td;
}

}  // namespace
