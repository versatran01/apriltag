#ifndef APRILTAG_MIT_H_
#define APRILTAG_MIT_H_

#include "AprilTags/TagDetector.h"
#include "AprilTags/TagDetection.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag16h5.h"
#include <boost/shared_ptr.hpp>

namespace AprilTags {

using TagDetectorPtr = boost::shared_ptr<TagDetector>;
using FloatPair = std::pair<float, float>;

}  // namespace apriltag_mit

namespace apriltag_mit = AprilTags;

#endif  // APRILTAG_MIT_H_
