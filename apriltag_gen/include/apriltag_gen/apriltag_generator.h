#ifndef APRILTAG_GEN_APRILTAG_GENERATOR_H_
#define APRILTAG_GEN_APRILTAG_GENERATOR_H_

#include <apriltag_mit/apriltag_mit.h>
#include <opencv2/core/core.hpp>
#include <apriltag_msgs/Apriltag.h>

namespace apriltag_gen {

class ApriltagGenerator {
 public:
  ApriltagGenerator();
  explicit ApriltagGenerator(const apriltag_mit::TagCodes& tag_codes);

  cv::Mat tagCode();

 private:
  apriltag_mit::TagFamily tag_familiy_;
};

cv::Mat tagCodeToCvMat(unsigned long long tag_code, int tag_bits,
                       int black_border = 1);

}  // namespace apriltag_gen

#endif  // APRILTAG_GEN_APRILTAG_GENERATOR_H_
