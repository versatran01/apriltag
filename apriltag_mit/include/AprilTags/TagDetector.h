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
  const TagFamily thisTagFamily;

  //! Constructor
  // note: TagFamily is instantiated here from TagCodes
  explicit TagDetector(const TagCodes& tagCodes, int blackBorder = 1)
      : thisTagFamily(tagCodes), blackBorder_(blackBorder) {}

  std::vector<TagDetection> extractTags(const cv::Mat& image);

  void setBlackBorder(int blackBorder) { this->blackBorder_ = blackBorder; }
  int getBlackBorder() const { return blackBorder_; }

 private:
  int blackBorder_;
};

}  // namespace AprilTags

#endif  // APRILTAGS_TAGDETECTOR_H_
