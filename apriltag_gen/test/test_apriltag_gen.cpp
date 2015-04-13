#include "apriltag_gen/apriltag_gen.h"
#include <gtest/gtest.h>

using namespace apriltag_gen;

TEST(TagGenTest, TagCodeToCvMat) {
  unsigned long long tag_code = 0x231bLL;
  int tag_bits = 4;
  const cv::Mat tag_matrix = tagCodeToCvMat(tag_code, tag_bits, 0);
  const cv::Mat result = (cv::Mat_<uchar>(tag_bits, tag_bits) << 0, 0, 1, 0, 0,
                          0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1) * 255;
  bool eq = std::equal(tag_matrix.begin<uchar>(), tag_matrix.end<uchar>(),
                       result.begin<uchar>());
  EXPECT_TRUE(eq);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
