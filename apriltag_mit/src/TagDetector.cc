#include <algorithm>
#include <cmath>
#include <climits>
#include <map>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>

#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/GrayModel.h"
#include "AprilTags/Line2D.h"
#include "AprilTags/LineSegment2D.h"
#include "AprilTags/Gridder.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/Segment.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/UnionFindSimple.h"
#include "AprilTags/XYWeight.h"

#include "AprilTags/TagDetector.h"

#include "AprilTags/timer.h"

namespace AprilTags {

using namespace std;

TagDetector::TagDetector(const TagCodes &tag_codes, int black_border)
    : tag_family_(tag_codes), black_border_(black_border) {}

void TagDetector::set_black_border(int black_border) {
  black_border_ = black_border;
}
int TagDetector::black_border() const { return black_border_; }

int TagDetector::CalcFilterSize(float sigma) const {
  return static_cast<int>(std::max(3.0f, 3 * sigma)) | 1;
}

int TagDetector::QuadLengthBits() const {
  return 2 * black_border_ + tag_family_.dimension_bits();
}

void TagDetector::Preprocess(const FloatImage &image, FloatImage &im_decode,
                             FloatImage &im_segment) const {
  im_decode = image;
  if (decode_sigma_ > 0) {
    const auto filter_size = CalcFilterSize(decode_sigma_);
    im_decode.FilterGaussian(filter_size, decode_sigma_);
  }

  if (segment_sigma_ > 0) {
    if (segment_sigma_ == decode_sigma_) {
      im_segment.mat() = im_decode.mat();
    } else {
      const auto filter_size = CalcFilterSize(segment_sigma_);
      // Copy
      im_segment = image;
      im_segment.FilterGaussian(filter_size, segment_sigma_);
    }
  } else {
    im_segment.mat() = image.mat();
  }
}

void TagDetector::CalcPolar(const FloatImage &image, FloatImage &im_mag,
                            FloatImage &im_theta) const {
  cv::Mat Ix, Iy;
  // Need to scale the gradient magnitude, because Scharr is 3, 10, 3
  cv::Scharr(image.mat(), Ix, CV_32F, 1, 0, 1 / 16.0);
  cv::Scharr(image.mat(), Iy, CV_32F, 0, 1, 1 / 16.0);
  cv::cartToPolar(Ix, Iy, im_mag.mat(), im_theta.mat());
}

void TagDetector::ChainSegments(std::vector<Segment> &segments,
                                const FloatImage &image) const {
  const int width = image.width();
  const int height = image.height();
  Gridder<Segment> gridder(width, height, 10);

  // add every segment to the hash table according to the position of the
  // segment's first point. Remember that the first point has a specific meaning
  // due to our left-hand rule above.
  for (Segment &s : segments) {
    gridder.Add(s.x0(), s.y0(), &s);
  }

  // Now, find child segments that begin where each parent segment ends.
  for (Segment &parent_seg : segments) {
    // compute length of the line segment
    Line2D parent_line(parent_seg);

    Gridder<Segment>::iterator iter = gridder.find(
        parent_seg.x1(), parent_seg.y1(), 0.5f * parent_seg.length());

    while (iter.hasNext()) {
      Segment &child_seg = iter.next();

      // We only look for segments that goes counterclockwise
      if (mod2pi(child_seg.theta() - parent_seg.theta()) > 0) {
        continue;
      }

      // compute intersection of points
      Line2D child_line(child_seg);

      auto p = parent_line.IntersectionWidth(child_line);
      if (p.x == -1) {
        continue;
      }

      float parent_dist = Distance2D(p, parent_seg.p1());
      float child_dist = Distance2D(p, child_seg.p0());

      if (max(parent_dist, child_dist) > parent_seg.length()) {
        continue;
      }

      // everything's OK, this child is a reasonable successor.
      parent_seg.children.push_back(&child_seg);
    }
  }
}

std::vector<Quad> TagDetector::SearchQuads(
    std::vector<Segment> &segments) const {
  std::vector<Quad> quads;

  // TODO: need to look at this
  std::vector<Segment *> tmp(5);
  for (size_t i = 0; i < segments.size(); i++) {
    tmp[0] = &segments[i];
    Quad::Search(tmp, segments[i], 0, quads);
  }

  return quads;
}

std::vector<TagDetection> TagDetector::DecodeQuads(
    const std::vector<Quad> &quads, const FloatImage &image) const {
  std::vector<TagDetection> detections;

  for (const Quad &quad : quads) {
    const auto td = tag_family_.DecodeQuad(quad, image, black_border());
    if (td.good) detections.push_back(td);
  }

  return detections;
}

std::vector<TagDetection> TagDetector::ResolveOverlap(
    const std::vector<TagDetection> &detections) const {
  std::vector<TagDetection> good_detections;
  good_detections.reserve(detections.size());

  for (const TagDetection &td : detections) {
    bool new_tag = true;

    for (TagDetection &gtd : good_detections) {
      // Tags have different id and they don't overlap too much
      if (td.id != gtd.id || !td.OverlapsTooMuch(gtd)) {
        continue;
      }

      // There's a conflict. We must pick one to keep.
      new_tag = false;

      // This detection is worse than the overlap one... just don't use it.
      if (td.hamming_distance > gtd.hamming_distance) {
        continue;
      }

      // Otherwise, keep the new one if it either has strictly *lower* error, or
      // greater perimeter.
      if (td.hamming_distance < gtd.hamming_distance ||
          td.obs_perimeter > gtd.obs_perimeter)
        gtd = td;
    }

    if (new_tag) {
      good_detections.push_back(td);
    }
  }

  return good_detections;
}

std::vector<TagDetection> TagDetector::ExtractTags(const cv::Mat &image) const {
  int width = image.cols;
  int height = image.rows;
  FloatImage im_orig(image);

  // ===========================================================================
  // Step 1: Preprocess image
  // ===========================================================================
  TimerUs t_step1("Preprocess");
  FloatImage im_decode, im_segment;
  Preprocess(im_orig, im_decode, im_segment);
  t_step1.stop();
  t_step1.report();

  // ===========================================================================
  // Step 2: Compute local gradients and store magnitude and direction
  // ===========================================================================
  // This step is quite sensitive to noise, since a few bad theta estimates will
  // break up segments, causing us to miss Quads. It is useful to do a Gaussian
  // low pass on this step even if we don't want if for encoding.
  // NOTE: in Kaess' original code, mag is Ix^2 + Iy^2, so we need to modify
  // Edge::kMinMag to reflect this change accordingly
  TimerUs t_step2("CalcPolar");
  FloatImage im_mag, im_theta;
  CalcPolar(im_segment, im_mag, im_theta);
  t_step2.stop();
  t_step2.report();

  //============================================================================
  // Step three. Extract edges by grouping pixels with similar thetas together.
  //============================================================================
  // This is a greedy algorithm: we start with the most similar pixels. We use
  // 4-connectivity.
  // NOTE: This is the most time consuming part!
  TimerUs t_step3("ExtractEdges");
  const size_t num_pixels = width * height;
  UnionFindSimple uf(num_pixels);

  vector<Edge> edges(num_pixels * 4);
  size_t num_edges = 0;

  // Bounds on the thetas assigned to this group. Note that because
  // theta is periodic, these are defined such that the average
  // value is contained *within* the interval.
  {
    // limit scope of storage
    // Previously all this was on the stack, but this is 1.2MB for 320x240
    // images That's already a problem for OS X (default 512KB thread stack
    // size), could be a problem elsewhere for bigger images... so store on heap

    // do all the memory in one big block, exception safe
    vector<float> storage(num_pixels * 4);
    float *theta_min = &storage[num_pixels * 0];
    float *theta_max = &storage[num_pixels * 1];
    float *mag_min = &storage[num_pixels * 2];
    float *mag_max = &storage[num_pixels * 3];

    for (int y = 0; y + 1 < height; y++) {
      for (int x = 0; x + 1 < width; x++) {
        const auto mag0 = im_mag.get(x, y);
        if (mag0 < Edge::kMinMag) {
          continue;
        }
        const int id = y * width + x;
        mag_max[id] = mag0;
        mag_min[id] = mag0;

        const auto theta0 = im_theta.get(x, y);
        theta_min[id] = theta0;
        theta_max[id] = theta0;

        // Calculates then adds edges to 'vector<Edge> edges'
        Edge::calcEdges(theta0, x, y, im_theta, im_mag, edges, num_edges);

        // XXX Would 8 connectivity help for rotated tags?
        // Probably not much, so long as input filtering hasn't been disabled.
      }
    }

    edges.resize(num_edges);
    std::stable_sort(edges.begin(), edges.end());
    Edge::mergeEdges(edges, uf, theta_min, theta_max, mag_min, mag_max);
  }
  t_step3.stop();
  t_step3.report();

  // ===========================================================================
  // Step 4: Loop over pixels to collect stattistics for each cluster
  // ===========================================================================
  // We will soon fit lines (segments) to these points.

  TimerUs t_step4("ClusterPixels");

  map<int, vector<XYW>> clusters;
  for (int y = 0; y < height - 1; ++y) {
    for (int x = 0; x < width - 1; ++x) {
      const int id = y * width + x;
      if (uf.getSetSize(id) < Segment::kMinSegmentPixels) {
        continue;
      }

      int rep = uf.getRepresentative(id);

      map<int, vector<XYW>>::iterator it = clusters.find(rep);
      if (it == clusters.end()) {
        clusters[rep] = vector<XYW>();
        it = clusters.find(rep);
      }
      vector<XYW> &points = it->second;
      points.push_back(XYW(x, y, im_mag.get(x, y)));
    }
  }
  t_step4.stop();
  t_step4.report();

  // ===========================================================================
  // Step 5: Loop over the clusters, fitting lines (which we call Segments).
  // ===========================================================================
  TimerUs t_step5("FitLine");
  std::vector<Segment> segments;  // used in Step six
  std::map<int, std::vector<XYW>>::const_iterator clustersItr;
  for (clustersItr = clusters.begin(); clustersItr != clusters.end();
       clustersItr++) {
    std::vector<XYW> xyws = clustersItr->second;
    LineSegment2D lseg = LineSegment2D::LsqFitXyw(xyws);

    // filter short lines
    float length = Distance2D(lseg.p0(), lseg.p1());
    if (length < Segment::kMinLineLength) continue;

    Segment seg;
    float dy = lseg.p1().y - lseg.p0().y;
    float dx = lseg.p1().x - lseg.p0().x;

    float tmpTheta = std::atan2(dy, dx);

    seg.set_theta(tmpTheta);
    seg.set_length(length);

    // We add an extra semantic to segments: the vector
    // p1->p2 will have dark on the left, white on the right.
    // To do this, we'll look at every gradient and each one
    // will vote for which way they think the gradient should
    // go. This is way more retentive than necessary: we
    // could probably sample just one point!

    float flip = 0, noflip = 0;
    for (size_t i = 0; i < xyws.size(); i++) {
      XYW xyw = xyws[i];

      float theta = im_theta.get((int)xyw.x, (int)xyw.y);
      float mag = im_mag.get((int)xyw.x, (int)xyw.y);

      // err *should* be +M_PI/2 for the correct winding, but if we
      // got the wrong winding, it'll be around -M_PI/2.
      float err = mod2pi(theta - seg.theta());

      if (err < 0)
        noflip += mag;
      else
        flip += mag;
    }

    if (flip > noflip) {
      float temp = seg.theta() + (float)M_PI;
      seg.set_theta(temp);
    }

    float dot = dx * std::cos(seg.theta()) + dy * std::sin(seg.theta());
    if (dot > 0) {
      seg.set_p0(lseg.p1());
      seg.set_p1(lseg.p0());
    } else {
      seg.set_p0(lseg.p0());
      seg.set_p1(lseg.p1());
    }

    segments.push_back(seg);
  }
  t_step5.stop();
  t_step5.report();

  // ===========================================================================
  // Step 6: For each segment, find segments that begin where this segment ends.
  // ===========================================================================
  // We will chain segments together next. The gridder accelerates the search by
  // building (essentially) a 2D hash table.

  TimerUs t_step6("ChainSegments");
  ChainSegments(segments, image);
  t_step6.stop();
  t_step6.report();

  //============================================================================
  // Step 7: Search all connected segments for quads.
  //============================================================================
  // To see if any form a loop of length 4.

  TimerUs t_step7("SearchQuads");
  const auto quads = SearchQuads(segments);
  t_step7.stop();
  t_step7.report();

  //============================================================================
  // Step 8: Decode the quads.
  //============================================================================
  // For each quad, we first estimate a threshold color to decide between 0 and
  // 1. Then, we read off the bits and see if they make sense.

  TimerUs t_step8("DecodeQuads");
  const auto detections = DecodeQuads(quads, im_decode);
  t_step8.stop();
  t_step8.report();

  // ===========================================================================
  // Step 9: Resolve overlapped tags.
  // ===========================================================================
  // Some quads may be detected more than once, due to
  // partial occlusion and our aggressive attempts to recover from
  // broken lines. When two quads (with the same id) overlap, we will
  // keep the one with the lowest error, and if the error is the same,
  // the one with the greatest observed perimeter.
  // NOTE: allow multiple non-overlapping detections of the same target.

  TimerUs t_step9("ResolveOverlap");
  const auto good_detections = ResolveOverlap(detections);
  t_step9.stop();
  t_step9.report();

  return good_detections;
}

}  // namespace
