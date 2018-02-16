#include "apriltag_ros/apriltag_detector.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace bpo = boost::program_options;
using namespace apriltag_ros;

int main(int argc, char **argv) {
  int type;
  int family;
  int decimate;
  bool refine;
  int black_border;

  bpo::options_description opt_desc("Options");
  opt_desc.add_options()("help", "Show help text.")(
      "type,t", bpo::value<int>(&type)->default_value(0),
      "Detector type, 0 - Mit, 1 - Umich")(
      "family,f", bpo::value<int>(&family)->default_value(0),
      "Tag family, 0 - tf36h11, 1 - tf25h9, 2 - tf16h5")(
      "decimate,d", bpo::value<int>(&decimate)->default_value(1),
      "Decimate image, 1 - no decimation")(
      "border,b", bpo::value<int>(&black_border)->default_value(1),
      "Black border around the tag")("image,i",
                                     bpo::value<std::vector<std::string>>(),
                                     "Images to detect apriltags on");
  bpo::positional_options_description pos_desc;
  pos_desc.add("image", -1);

  bpo::variables_map var_map;
  try {
    bpo::store(bpo::command_line_parser(argc, argv)
                   .options(opt_desc)
                   .positional(pos_desc)
                   .run(),
               var_map);
    bpo::notify(var_map);
  } catch (const std::exception &e) {
    std::cout << e.what() << "\n\n";
    return 1;
  }

  // Handle all options
  if (var_map.count("help")) {
    std::cout << opt_desc << "\n";
    return 0;
  }

  ApriltagDetectorPtr detector;
  try {
    detector = ApriltagDetector::Create(static_cast<DetectorType>(type),
                                        static_cast<TagFamily>(family));
    detector->set_black_border(black_border);
  } catch (const std::exception &e) {
    std::cout << e.what() << "\n";
    return 1;
  }

  const auto &images = var_map["image"].as<std::vector<std::string>>();
  for (const auto &imfile : images) {
    std::cout << imfile << " ... ";
    const auto gray = cv::imread(imfile, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      std::cout << "failed to read image\n";
      continue;
    }

    // detect
    auto apriltags = detector->Detect(gray);
    std::cout << apriltags.size() << " tags";

    // draw
    cv::Mat disp;
    cv::cvtColor(gray, disp, cv::COLOR_GRAY2BGR);
    DrawApriltags(disp, apriltags);

    // visualize
    cv::namedWindow(imfile, cv::WINDOW_NORMAL);
    cv::imshow(imfile, disp);
    cv::waitKey(-1);
    std::cout << std::endl;
  }
}
