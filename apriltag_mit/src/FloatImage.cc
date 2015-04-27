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

void FloatImage::filterFactoredCentered(const std::vector<float> &fhoriz,
                                        const std::vector<float> &fvert) {
  // do horizontal
  std::vector<float> r(pixels);

  for (int y = 0; y < height; y++) {
    Gaussian::convolveSymmetricCentered(pixels, y * width, width, fhoriz, r,
                                        y * width);
  }

  // do vertical
  std::vector<float> tmp(height);   // column before convolution
  std::vector<float> tmp2(height);  // column after convolution

  for (int x = 0; x < width; x++) {
    // copy the column out for locality
    for (int y = 0; y < height; y++) tmp[y] = r[y * width + x];

    Gaussian::convolveSymmetricCentered(tmp, 0, height, fvert, tmp2, 0);

    for (int y = 0; y < height; y++) pixels[y * width + x] = tmp2[y];
  }
}

}  // namespace
