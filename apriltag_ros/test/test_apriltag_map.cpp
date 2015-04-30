#include "apriltag_ros/apriltag_map.h"

using namespace apriltag_ros;
int main() {
  const std::string filename(
      "/home/chao/Workspace/repo/versatran01/sv_apriltag/"
      "apriltag_gen/map/april_grid.yaml");
  std::cout << "filename: " << filename << std::endl;
  loadApriltagMapYaml(filename);
}
