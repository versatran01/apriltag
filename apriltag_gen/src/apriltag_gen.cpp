#include "apriltag_gen/apriltag_gen.h"

namespace apriltag_gen {

cv::Mat tagCodeToCvMat(unsigned long long tag_code, int tag_bits,
                       int black_border) {
  cv::Mat tag_matrix = cv::Mat::zeros(tag_bits + black_border * 2,
                                      tag_bits + black_border * 2, CV_8UC1);
  for (int i = 0; i < tag_bits; ++i) {
    uchar* p = tag_matrix.ptr<uchar>(i + black_border);
    for (int j = 0; j < tag_bits; ++j) {
      int shift = tag_bits * (tag_bits - i) - j - 1;
      if (tag_code & (1 << shift)) {
        p[j + black_border] = 255;
      }
    }
  }

  return tag_matrix;
}

}  // namespace apriltag_gen
