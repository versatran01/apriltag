#ifndef APRILTAG_MIT_H_
#define APRILTAG_MIT_H_

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag16h5.h"

#include <memory>

namespace apriltag_mit = AprilTags;
namespace AprilTags {

using TagDetectorPtr = std::unique_ptr<TagDetector>;
using FloatPair = std::pair<float, float>;

}  // namespace apriltag_mit

#endif  // APRILTAG_MIT_H_
