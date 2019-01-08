#pragma once

extern "C" {
#include "apriltag.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
// Not supported for now
//#include "tagCircle21h7.h"
//#include "tagCircle49h12.h"
//#include "tagCustom48h12.h"
//#include "tagStandard41h12.h"
//#include "tagStandard52h13.h"
}

#include <memory>

namespace apriltag {

struct FreeFamily {
  void operator()(apriltag_family_t *tf) const {
    free(tf->codes);
    free(tf->bit_x);
    free(tf->bit_y);
    free(tf);
  }
};

struct FreeDetector {
  void operator()(apriltag_detector_t *td) const {
    apriltag_detector_destroy(td);
  }
};

struct FreeDetectionZa {
  void operator()(zarray_t *za) const { apriltag_detections_destroy(za); }
};

using FamilyUPtr = std::unique_ptr<apriltag_family_t, FreeFamily>;
using DetectorUPtr = std::unique_ptr<apriltag_detector_t, FreeDetector>;
using DetectionZaUPtr = std::unique_ptr<zarray_t, FreeDetectionZa>;

}  // namespace apriltag
