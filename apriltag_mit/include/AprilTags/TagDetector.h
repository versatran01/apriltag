#ifndef APRILTAGS_TAGDETECTOR_H_
#define APRILTAGS_TAGDETECTOR_H_

#include <opencv2/core/core.hpp>

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/Quad.h"
#include "AprilTags/GrayModel.h"

namespace AprilTags {

class TagDetector {
 public:
  explicit TagDetector(const TagCodes& tag_codes, int black_border = 1);

  std::vector<TagDetection> ExtractTags(const cv::Mat& image) const;

  void set_black_border(int black_border);
  int black_border() const;

 private:
  const TagFamily tag_family_;

  int CalcFilterSize(float sigma) const;

  int QuadLengthBits() const;

  /**
   * @brief Preprocess Step 1
   * @param image
   * @param im_decode
   * @param im_segment
   */
  void Preprocess(const FloatImage& image, FloatImage& im_decode,
                  FloatImage& im_segment) const;

  /**
   * @brief CalcPolar Step 2
   * @param image
   * @param im_mag
   * @param im_theta
   */
  void CalcPolar(const FloatImage& image, FloatImage& im_mag,
                 FloatImage& im_theta) const;

  /**
   * @brief SearchQuads Step 7
   * @param segments
   * @return
   */
  std::vector<Quad> SearchQuads(std::vector<Segment>& segments) const;

  /**
   * @brief DecodeQuads Step 8
   * @param quads
   * @return
   */
  std::vector<TagDetection> DecodeQuads(const std::vector<Quad>& quads,
                                        const FloatImage& image) const;
  /**
   * @brief MakeGrayModel Step 8.1
   * @param quad
   * @param image
   * @return
   */
  GrayModel MakeGrayModel(const Quad& quad, const FloatImage& image) const;

  /**
   * @brief ResolveOverlap Step 9
   * @param detections
   * @return
   */
  std::vector<TagDetection> ResolveOverlap(
      const std::vector<TagDetection>& detections) const;

  /**
   * @brief black_border_ Number of bits of black border of the tag
   */
  unsigned black_border_ = 1;

  /**
   * @brief decode_sigma_ Gaussian smoothing kernel applied to image
   * Used when sampling bits. Filtering is a good idea in cases where A) a cheap
   * camera is introducing artificial sharpening, B) the bayer pattern is
   * creating artifacts, C) the sensor is very noisy and/or has hot/cold pixels.
   * However, filtering makes it harder to decode very small tags. Resonable
   * values are 0, 0.8, 1.5
   */
  float decode_sigma_ = 0.8;

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
