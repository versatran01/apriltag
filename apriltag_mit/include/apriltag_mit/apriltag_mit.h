#pragma once

#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/TagDetection.h"
#include "AprilTags/TagDetector.h"

#include <boost/shared_ptr.hpp>

namespace AprilTags {

using TagDetectorPtr = boost::shared_ptr<TagDetector>;
using FloatPair = std::pair<float, float>;

} // namespace apriltag_mit

namespace apriltag_mit = AprilTags;
