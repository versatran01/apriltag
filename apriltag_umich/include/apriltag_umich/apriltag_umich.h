#ifndef APRILTAG_UMICH_H_
#define APRILTAG_UMICH_H_

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/zarray.h"
}

#include <memory>

namespace apriltag_umich {

struct FreeTagFamily {
  void operator()(apriltag_family_t *tf) const {
    free(tf->codes);
    free(tf);
  }
};

struct FreeTagDetector {
  void operator()(apriltag_detector_t *td) const {
    apriltag_detector_destroy(td);
  }
};

struct FreeTagDetection {
  void operator()(apriltag_detection_t *det) const {
    apriltag_detection_destroy(det);
  }
};

struct FreeImageu8 {
  void operator()(image_u8_t *im) const { image_u8_destroy(im); }
};

struct FreeZarray {
  void operator()(zarray_t *za) const { zarray_destroy(za); }
};

using TagFamilyPtr = std::unique_ptr<apriltag_family_t, FreeTagFamily>;
using TagDetectorPtr = std::unique_ptr<apriltag_detector_t, FreeTagDetector>;
using TagDetectionPtr = std::unique_ptr<apriltag_detection_t, FreeTagDetection>;
using Imageu8Ptr = std::unique_ptr<image_u8_t, FreeImageu8>;
using ZarrayPtr = std::unique_ptr<zarray_t, FreeZarray>;

}

#endif  // APRILTAG_UMICH_H_
