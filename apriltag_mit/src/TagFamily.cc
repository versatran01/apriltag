#include <iostream>

#include "AprilTags/TagFamily.h"
#include <numeric>

/**

// example of instantiation of tag family:

#include "AprilTags/TagFamily.h"
#include "AprilTags/Tag36h11.h"
TagFamily *tag36h11 = new TagFamily(tagCodes36h11);

// available tag families:

#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag16h5_other.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag36h11_other.h"
#include "AprilTags/Tag36h9.h"

*/

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

void TagFamily::decode(TagDetection &det, code_t obs_code) const {
  int best_id = -1;
  int best_hamming = std::numeric_limits<int>::max();
  int best_rotation = 0;
  code_t best_code = 0;

  code_t rot_codes[4];
  rot_codes[0] = obs_code;
  rot_codes[1] = rotate90_cwise(rot_codes[0], dimension_bits());
  rot_codes[2] = rotate90_cwise(rot_codes[1], dimension_bits());
  rot_codes[3] = rotate90_cwise(rot_codes[2], dimension_bits());

  for (unsigned int id = 0; id < num_codes_; ++id) {
    for (unsigned int rot = 0; rot < 4; ++rot) {
      int hamming = hamming_distance(rot_codes[rot], codes()[id]);
      if (hamming < best_hamming) {
        best_hamming = hamming;
        best_rotation = rot;
        best_id = id;
        best_code = codes()[id];
      }
    }
  }
  det.id = best_id;
  det.hamming_distance = best_hamming;
  det.num_rotations = best_rotation;
  det.good = (det.hamming_distance <= error_recovery_bits_);
  det.obs_code = obs_code;
  det.code = best_code;
}

code_t rotate90_cwise(code_t w, int d) {
  code_t wr = 0;
  const code_t oneLongLong = 1;

  for (int r = d - 1; r >= 0; r--) {
    for (int c = 0; c < d; c++) {
      int b = r + d * c;
      wr = wr << 1;

      if ((w & (oneLongLong << b)) != 0) wr |= 1;
    }
  }
  return wr;
}

unsigned hamming_distance(code_t a, code_t b) {
  // Because code_t is unsigned long long
  return __builtin_popcountll(a ^ b);
}

}  // namespace AprilTags
