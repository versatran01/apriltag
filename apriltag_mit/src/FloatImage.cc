#include "AprilTags/FloatImage.h"
#include "AprilTags/Gaussian.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

namespace AprilTags {

FloatImage::FloatImage(int widthArg, int heightArg) {
  image_.create(heightArg, widthArg, CV_32FC1);
}

FloatImage &FloatImage::operator=(const FloatImage &other) {
  other.image_.copyTo(image_);
  return *this;
}

void FloatImage::filterFactoredCentered(int ksize, float sigma) {
  cv::GaussianBlur(image_, image_, cv::Size(ksize, ksize), sigma);
}

}  // namespace
