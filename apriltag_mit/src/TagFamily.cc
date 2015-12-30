#include <iostream>
#include <numeric>

#include "AprilTags/TagFamily.h"
#include "AprilTags/MathUtil.h"

namespace AprilTags {

using namespace std;

TagFamily::TagFamily(const TagCodes &tag_codes)
    : tag_codes_(tag_codes),
      num_codes_(tag_codes.codes.size()),
      error_recovery_bits_(1) {}

unsigned TagFamily::payload_bits() const { return tag_codes_.payload_bits; }
unsigned TagFamily::dimension_bits() const { return tag_codes_.dimension_bits; }
unsigned TagFamily::min_hamming() const { return tag_codes_.min_hamming; }
const std::vector<code_t> &TagFamily::codes() const { return tag_codes_.codes; }
size_t TagFamily::num_codes() const { return num_codes(); }

void TagFamily::set_error_recovery_bits(unsigned error_recovery_bits) {
  error_recovery_bits_ = error_recovery_bits;
}
void TagFamily::set_error_recovery_fraction(float v) {
  error_recovery_bits_ = static_cast<unsigned>(((min_hamming() - 1) / 2) * v);
}

code_t TagFamily::Code(unsigned id) const { return codes()[id]; }

bool TagFamily::IsGood(unsigned id, unsigned hamming_distance) const {
  return (id != std::numeric_limits<unsigned>::max()) &&
         (hamming_distance <= error_recovery_bits_);
}

TagDetection TagFamily::Decode(code_t obs_code) const {
  auto best_id = std::numeric_limits<unsigned>::max();
  auto best_hamming = std::numeric_limits<unsigned>::max();
  unsigned best_rotation = 0;

  code_t rot_codes[4];
  rot_codes[0] = obs_code;
  rot_codes[1] = Rotate90Cwise(rot_codes[0], dimension_bits());
  rot_codes[2] = Rotate90Cwise(rot_codes[1], dimension_bits());
  rot_codes[3] = Rotate90Cwise(rot_codes[2], dimension_bits());

  for (size_t id = 0; id < num_codes_; ++id) {
    for (size_t rot = 0; rot < 4; ++rot) {
      const auto hamming = HammingDistance(rot_codes[rot], Code(id));
      if (hamming == 0) {
        // Hamming distance is 0, this is our tag, no need to test against other
        // tags
        return TagDetection(id, true, obs_code, Code(id), hamming, rot);
      } else if (hamming < best_hamming) {
        best_id = id;
        best_hamming = hamming;
        best_rotation = rot;
      }
    }
  }

  return TagDetection(best_id, IsGood(best_id, best_hamming), obs_code,
                      Code(best_id), best_hamming, best_rotation);
}

TagDetection TagFamily::DecodeQuad(const Quad &quad, const FloatImage &image,
                                   unsigned black_border) const {
  const int lb = 2 * black_border + dimension_bits();
  const auto gray_model = quad.MakeGrayModel(image, lb);

  bool bad = false;
  code_t tag_code = 0;

  for (int yb = dimension_bits() - 1; yb >= 0; yb--) {
    float yn = (black_border + yb + 0.5f) / lb;
    for (int xb = 0; xb < dimension_bits(); xb++) {
      float xn = (black_border + xb + 0.5f) / lb;

      const auto pi = quad.Interpolate01({xn, yn});
      int xi = pi.x + 0.5;
      int yi = pi.y + 0.5;

      if (!IsInsideImage(xi, yi, image)) {
        bad = true;
        continue;
      }
      const float threshold = gray_model.CalcThreshold(xn, yn);
      float v = image.get(xi, yi);
      tag_code = tag_code << 1;
      if (v > threshold) {
        tag_code |= 1;
      }
    }
  }

  if (!bad) {
    auto td = Decode(tag_code);

    // compute the homography (and rotate it appropriately)
    td.H = CalcHomography(quad.p);

    float c = std::cos(td.num_rotations * Pi_2<float>());
    float s = std::sin(td.num_rotations * Pi_2<float>());
    auto R = cv::Matx33f::zeros();
    R(0, 0) = R(1, 1) = c;
    R(0, 1) = -s;
    R(1, 0) = s;
    R(2, 2) = 1;
    td.H = td.H * R;

    // Rotate points in detection according to decoded orientation. Thus the
    // order of the points in the detection object can be used to determine
    // the orientation of the target.
    const auto bl = td.Project({-1, -1});
    int best_rot = -1;
    float best_dist = FLT_MAX;
    for (size_t i = 0; i < 4; ++i) {
      const float dist = Distance2D(bl, quad.p[i]);
      if (dist < best_dist) {
        best_dist = dist;
        best_rot = i;
      }
    }

    for (size_t i = 0; i < 4; ++i) {
      td.p[i] = quad.p[(i + best_rot) % 4];
    }
    return td;
  } else {
    return TagDetection();
  }
}

code_t Rotate90Cwise(code_t w, int d) {
  code_t wr = 0;
  const code_t one = 1;

  for (int r = d - 1; r >= 0; r--) {
    for (int c = 0; c < d; c++) {
      int b = r + d * c;
      wr = wr << 1;

      if ((w & (one << b)) != 0) wr |= 1;
    }
  }
  return wr;
}

unsigned HammingDistance(code_t a, code_t b) {
  // Because code_t is unsigned long long
  return __builtin_popcountll(a ^ b);
}

}  // namespace AprilTags
