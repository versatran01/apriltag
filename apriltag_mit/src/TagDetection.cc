#include "opencv2/opencv.hpp"
#include <iterator>
#include "AprilTags/TagDetection.h"
#include "AprilTags/MathUtil.h"

namespace AprilTags {

// float TagDetection::getXYOrientation() const {
//  // Because the order of segments in a quad is arbitrary, so is the
//  // homography's rotation, so we can't determine orientation directly
//  // from the homography.  Instead, use the homography to find two
//  // bottom corners of a properly oriented tag in pixel coordinates,
//  // and then compute orientation from that.
//  std::pair<float, float> p0 = interpolate(-1, -1);  // lower left corner of
//  tag
//  std::pair<float, float> p1 = interpolate(1, -1);  // lower right corner of
//  tag
//  float orient = atan2(p1.second - p0.second, p1.first - p0.first);
//  return !std::isnan(float(orient)) ? orient : 0.;
//}

std::pair<float, float> TagDetection::interpolate(float x, float y) const {
  float z = H(2, 0) * x + H(2, 1) * y + H(2, 2);
  // prevents returning a pair with -NaN, for which gcc 4.4 flubs isnan
  if (z == 0) return std::pair<float, float>(0, 0);

  float newx = (H(0, 0) * x + H(0, 1) * y + H(0, 2)) / z;
  float newy = (H(1, 0) * x + H(1, 1) * y + H(1, 2)) / z;
  return std::pair<float, float>(newx, newy);
}

bool TagDetection::OverlapsTooMuch(const TagDetection &other) const {
  // Compute a sort of "radius" of the two targets. We'll do this by
  // computing the average length of the edges of the quads (in
  // pixels).
  const float radius =
      (Distance2D(p[0], p[1]) + Distance2D(p[1], p[2]) +
       Distance2D(p[2], p[3]) + Distance2D(p[3], p[0]) +
       Distance2D(other.p[0], other.p[1]) + Distance2D(other.p[1], other.p[2]) +
       Distance2D(other.p[2], other.p[3]) +
       Distance2D(other.p[3], other.p[0])) /
      16.0f;

  // distance (in pixels) between two tag centers
  const auto dist = Distance2D(cxy, other.cxy);

  // reject pairs where the distance between centroids is smaller than
  // the "radius" of one of the tags.
  return (dist < radius);
}

void TagDetection::ScaleTag(float scale) {
  cxy.x *= scale;
  cxy.y *= scale;
  for (cv::Point2f &c : p) {
    c.x *= scale;
    c.y *= scale;
  }
}

// Eigen::Matrix4d TagDetection::getRelativeTransform(double tag_size, double
// fx,
//                                                   double fy, double px,
//                                                   double py) const {
//  float s = tag_size / 2.;
/*
std::vector<cv::Point3f> objPts;
std::vector<cv::Point2f> imgPts;
objPts.push_back(cv::Point3f(-s, -s, 0));
objPts.push_back(cv::Point3f(s, -s, 0));
objPts.push_back(cv::Point3f(s, s, 0));
objPts.push_back(cv::Point3f(-s, s, 0));

std::pair<float, float> p1 = p[0];
std::pair<float, float> p2 = p[1];
std::pair<float, float> p3 = p[2];
std::pair<float, float> p4 = p[3];
imgPts.push_back(cv::Point2f(p1.first, p1.second));
imgPts.push_back(cv::Point2f(p2.first, p2.second));
imgPts.push_back(cv::Point2f(p3.first, p3.second));
imgPts.push_back(cv::Point2f(p4.first, p4.second));

std::for_each(begin(p), end(p), [&imgPts](const Pointf &corner) {
  imgPts.emplace_back(corner.first, corner.second);
});
*/

//  std::vector<cv::Point3f> objPts = {
//      {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}};
//  std::vector<cv::Point2f> imgPts = {{p[0].first, p[0].second},
//                                     {p[1].first, p[1].second},
//                                     {p[2].first, p[2].second},
//                                     {p[3].first, p[3].second}};

//  cv::Mat rvec, tvec;
//  cv::Matx33f cameraMatrix(fx, 0, px, 0, fy, py, 0, 0, 1);
//  cv::Vec4f distParam(0, 0, 0, 0);  // all 0?
//  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
//  cv::Matx33d r;
//  cv::Rodrigues(rvec, r);
//  Eigen::Matrix3d wRo;
//  wRo << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2,
//  1),
//      r(2, 2);

//  Eigen::Matrix4d T;
//  T.topLeftCorner(3, 3) = wRo;
//  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1),
//      tvec.at<double>(2);
//  T.row(3) << 0, 0, 0, 1;

//  return T;
//}

// void TagDetection::getRelativeQT(double tag_size, const cv::Matx33d &K,
//                                 const cv::Mat_<double> &D,
//                                 Eigen::Quaterniond &quat,
//                                 Eigen::Vector3d &trans) const {
//  cv::Mat rvec, tvec;
//  getRelativeRT(tag_size, K, D, rvec, tvec);
//  trans = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1),
//                          tvec.at<double>(2));
//  Eigen::Vector3d r(rvec.at<double>(0), rvec.at<double>(1),
//  rvec.at<double>(2));
//  // Copied from kr_math pose
//  const double rn = r.norm();
//  Eigen::Vector3d rnorm(0.0, 0.0, 0.0);
//  if (rn > std::numeric_limits<double>::epsilon() * 10) {
//    rnorm = r / rn;
//  }
//  quat = Eigen::AngleAxis<double>(rn, rnorm);
//}

// void TagDetection::getRelativeRT(double tag_size, const cv::Matx33d &K,
//                                 const cv::Mat_<double> &D, cv::Mat &rvec,
//                                 cv::Mat &tvec) const {
//  float s = tag_size / 2.;
//  // tag corners in tag frame, which we call object
//  std::vector<cv::Point3f> p_obj = {
//      {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}};
//  // pixels coordinates in image frame
//  std::vector<cv::Point2f> p_img = {{p[0].first, p[0].second},
//                                    {p[1].first, p[1].second},
//                                    {p[2].first, p[2].second},
//                                    {p[3].first, p[3].second}};
//  cv::solvePnP(p_obj, p_img, K, D, rvec, tvec);
//}

// Eigen::Matrix4d TagDetection::getRelativeH(double tag_size,
//                                           const cv::Matx33d &K,
//                                           const cv::Mat_<double> &D) const {
//  cv::Mat rvec, tvec;
//  getRelativeRT(tag_size, K, D, rvec, tvec);
//  cv::Matx33d r;  // This R is cRo
//  cv::Rodrigues(rvec, r);
//  Eigen::Matrix3d cRo;
//  cRo << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2,
//  1),
//      r(2, 2);
//  Eigen::Matrix4d cTo;
//  cTo.topLeftCorner(3, 3) = cRo;
//  cTo.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1),
//      tvec.at<double>(2);
//  cTo.row(3) << 0, 0, 0, 1;
//  return cTo;
//}

// void TagDetection::getRelativeTranslationRotation(double tag_size, double fx,
//                                                  double fy, double px,
//                                                  double py,
//                                                  Eigen::Vector3d &trans,
//                                                  Eigen::Matrix3d &rot) const
//                                                  {
//  Eigen::Matrix4d T = getRelativeTransform(tag_size, fx, fy, px, py);

//  // converting from camera frame (z forward, x right, y down) to
//  // object frame (x forward, y left, z up)
//  Eigen::Matrix4d M;
//  M << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
//  Eigen::Matrix4d MT = M * T;
//  // translation vector from camera to the April tag
//  trans = MT.col(3).head(3);
//  // orientation of April tag with respect to camera: the camera
//  // convention makes more sense here, because yaw,pitch,roll then
//  // naturally agree with the orientation of the object
//  rot = T.block(0, 0, 3, 3);
//}

}  // namespace
