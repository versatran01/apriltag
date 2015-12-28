#ifndef APRILTAGS_TAGCODES_H_
#define APRILTAGS_TAGCODES_H_

#include <vector>

namespace AprilTags {
class TagCodes {
 public:
  TagCodes(int bits, int min_hamming_distance, const unsigned long long* codesA,
           int num)
      : payload_bits_(bits),
        min_hamming_distance_(min_hamming_distance),
        codes(codesA, codesA + num) {}

 public:
  int payload_bits_;
  int min_hamming_distance_;
  std::vector<unsigned long long> codes;
};

}  // namespace AprilTags

#endif  // APRILTAGS_TAGCODES_H
