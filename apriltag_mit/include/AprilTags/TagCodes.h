#ifndef APRILTAGS_TAGCODES_H_
#define APRILTAGS_TAGCODES_H_

#include <vector>

using code_t = unsigned long long;

namespace AprilTags {
class TagCodes {
 public:
  TagCodes(int bits, int min_hamming_distance, const code_t* codesA, int num);

  int payload_bits() const;
  int min_hamming_distance() const;

  const std::vector<code_t>& codes() const;

 private:
  int payload_bits_;
  int min_hamming_distance_;
  std::vector<code_t> codes_;
};

}  // namespace AprilTags

#endif  // APRILTAGS_TAGCODES_H
