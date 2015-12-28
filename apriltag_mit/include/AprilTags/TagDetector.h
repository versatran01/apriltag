#ifndef APRILTAGS_TAGDETECTOR_H_
#define APRILTAGS_TAGDETECTOR_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"

namespace AprilTags {

class TagDetector {
 public:
  explicit TagDetector(const TagCodes& tag_codes, int black_border = 1);

  std::vector<TagDetection> ExtractTags(const cv::Mat& image) const;

  void set_black_border(int blackBorder);
  int black_border() const;

 private:
  const TagFamily tag_family_;
  /**
   * @brief black_border_ Number of bits of black border of the tag
   */
  int black_border_ = 1;

  /**
   * @brief sigma_ Gaussian smoothing kernel applied to image
   * Used when sampling bits. Filtering is a good idea in cases where A) a cheap
   * camera is introducing artificial sharpening, B) the bayer pattern is
   * creating artifacts, C) the sensor is very noisy and/or has hot/cold pixels.
   * However, filtering makes it harder to decode very small tags. Resonable
   * values are 0, 0.8, 1.5
   */
  float sampling_sigma_ = 0.8;

  /**
   * @brief segment_sigma_ Gaussian smoothing kernel applied to image
   * Used when detecting the outline of the box. It is almost always useful to
   * have some filtering, since the loss of small details won't hurt.
   * Recommended value is 0.8.
   */
  float segment_sigma_ = 0.8;
};

}  // namespace AprilTags

#endif  // APRILTAGS_TAGDETECTOR_H_
