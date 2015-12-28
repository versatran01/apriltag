#ifndef APRILTAGS_FLOATIMAGE_H_
#define APRILTAGS_FLOATIMAGE_H_

#include <algorithm>
#include <vector>
#include <opencv2/core/core.hpp>

namespace AprilTags {

//! Represent an image as a vector of floats in [0,1]
class FloatImage {
 private:
  cv::Mat image_;

 public:
  //! Default constructor
  FloatImage() = default;
  FloatImage(const cv::Mat& image);

  //! Construct an empty image
  FloatImage(int width, int height);

  FloatImage& operator=(const FloatImage& other);

  float get(int x, int y) const { return image_.at<float>(y, x); }
  void set(int x, int y, float v) { image_.at<float>(y, x) = v; }

  int width() const { return image_.cols; }
  int height() const { return image_.rows; }
  int num_pixels() const { return image_.cols * image_.rows; }

  void FilterGaussian(int ksize, float sigma);
};

}  // namespace AprilTags

#endif  // APRILTAGS_FLOATIMAGE_H_
