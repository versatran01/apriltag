#include <iostream>

#include "AprilTags/TagFamily.h"

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
    : payload_bits_(tag_codes.payload_bits()),
      dimension_bits_((int)std::sqrt((float)payload_bits_)),
      min_hamming_distance_(tag_codes.min_hamming_distance()),
      error_recovery_bits_(1),
      codes() {
  if (payload_bits_ != dimension_bits_ * dimension_bits_)
    cerr << "Error: TagFamily constructor called with bits=" << payload_bits_
         << "; must be a square number!" << endl;
  codes = tag_codes.codes();
}

void TagFamily::setErrorRecoveryBits(int b) { error_recovery_bits_ = b; }

void TagFamily::setErrorRecoveryFraction(float v) {
  error_recovery_bits_ = (int)(((int)(min_hamming_distance_ - 1) / 2) * v);
}

code_t TagFamily::rotate90(code_t w, int d) {
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

int TagFamily::hammingDistance(code_t a, code_t b) { return popCount(a ^ b); }

unsigned char TagFamily::popCountReal(code_t w) {
  unsigned char cnt = 0;
  while (w != 0) {
    w &= (w - 1);
    ++cnt;
  }
  return cnt;
}

int TagFamily::popCount(code_t w) {
  int count = 0;
  while (w != 0) {
    count += popCountTable[(unsigned int)(w & (popCountTableSize - 1))];
    w >>= popCountTableShift;
  }
  return count;
}

void TagFamily::decode(TagDetection &det, code_t rCode) const {
  int bestId = -1;
  int bestHamming = INT_MAX;
  int bestRotation = 0;
  code_t bestCode = 0;

  code_t rCodes[4];
  rCodes[0] = rCode;
  rCodes[1] = rotate90(rCodes[0], dimension_bits_);
  rCodes[2] = rotate90(rCodes[1], dimension_bits_);
  rCodes[3] = rotate90(rCodes[2], dimension_bits_);

  for (unsigned int id = 0; id < codes.size(); id++) {
    for (unsigned int rot = 0; rot < 4; rot++) {
      int thisHamming = hammingDistance(rCodes[rot], codes[id]);
      if (thisHamming < bestHamming) {
        bestHamming = thisHamming;
        bestRotation = rot;
        bestId = id;
        bestCode = codes[id];
      }
    }
  }
  det.id = bestId;
  det.hammingDistance = bestHamming;
  det.rotation = bestRotation;
  det.good = (det.hammingDistance <= error_recovery_bits_);
  det.obsCode = rCode;
  det.code = bestCode;
}

void TagFamily::printHammingDistances() const {
  vector<int> hammings(dimension_bits_ * dimension_bits_ + 1);
  for (unsigned i = 0; i < codes.size(); i++) {
    code_t r0 = codes[i];
    code_t r1 = rotate90(r0, dimension_bits_);
    code_t r2 = rotate90(r1, dimension_bits_);
    code_t r3 = rotate90(r2, dimension_bits_);
    for (unsigned int j = i + 1; j < codes.size(); j++) {
      int d = min(
          min(hammingDistance(r0, codes[j]), hammingDistance(r1, codes[j])),
          min(hammingDistance(r2, codes[j]), hammingDistance(r3, codes[j])));
      hammings[d]++;
    }
  }

  for (unsigned int i = 0; i < hammings.size(); i++)
    printf("hammings: %u = %d\n", i, hammings[i]);
}

unsigned char TagFamily::popCountTable[TagFamily::popCountTableSize];

TagFamily::TableInitializer TagFamily::initializer;

}  // namespace
