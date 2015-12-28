#include "AprilTags/TagCodes.h"

namespace AprilTags {

TagCodes::TagCodes(int bits, int min_hamming_distance, const code_t* codesA,
                   int num)
    : payload_bits_(bits),
      min_hamming_distance_(min_hamming_distance),
      codes_(codesA, codesA + num) {}

int TagCodes::payload_bits() const { return payload_bits_; }

int TagCodes::min_hamming_distance() const { return min_hamming_distance_; }

const std::vector<code_t>& TagCodes::codes() const { return codes_; }

}  // namespace AprilTags
