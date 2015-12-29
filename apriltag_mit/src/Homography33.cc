#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include "AprilTags/Homography33.h"

Homography33::Homography33() : fA(), H(), valid(false) {
  fA.setZero();
  H.setZero();
}

Eigen::Matrix3d &Homography33::getH() {
  compute();
  return H;
}

void Homography33::setCorrespondences(
    const std::vector<std::pair<float, float>> &sPts,
    const std::vector<std::pair<float, float>> &dPts) {
  valid = false;
  srcPts = sPts;
  dstPts = dPts;
}

void Homography33::compute() {
  if (valid) return;

  std::vector<cv::Point2f> sPts;
  std::vector<cv::Point2f> dPts;
  for (int i = 0; i < 4; i++) {
    sPts.push_back(cv::Point2f(srcPts[i].first, srcPts[i].second));
  }
  for (int i = 0; i < 4; i++) {
    dPts.push_back(cv::Point2f(dstPts[i].first, dstPts[i].second));
  }
  cv::Mat homography = cv::findHomography(sPts, dPts);
  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) {
      H(r, c) = homography.at<double>(r, c);
    }
  }

  valid = true;
}

// std::pair<float, float> Homography33::project(float worldx, float worldy) {
//  compute();

//  std::pair<float, float> ixy;
//  ixy.first = H(0, 0) * worldx + H(0, 1) * worldy + H(0, 2);
//  ixy.second = H(1, 0) * worldx + H(1, 1) * worldy + H(1, 2);
//  float z = H(2, 0) * worldx + H(2, 1) * worldy + H(2, 2);
//  ixy.first = ixy.first / z + cxy.first;
//  ixy.second = ixy.second / z + cxy.second;
//  return ixy;
//}
