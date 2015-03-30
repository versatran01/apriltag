#ifndef APRILTAG_GEN_APRILTAG_GEN_H_
#define APRILTAG_GEN_APRILTAG_GEN_H_

#include <opencv2/core/core.hpp>

namespace apriltag_gen {

cv::Mat tagCodeToCvMat(unsigned long long tag_code, int tag_bits,
                       int black_border = 1);

}  // namespace apriltag_gen

#endif  // APRILTAG_GEN_APRILTAG_GEN_H_
