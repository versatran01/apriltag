#include "apriltag_ros/apriltag_map.h"

namespace apriltag_ros {

void ApriltagMap::addTag(const Tag3D& tag) {
  if (tag_map_.find(tag.id()) == tag_map_.end()) {
    // Not found add to map
    tag_map_[tag.id()] = tag;
  } else {
    // Already there, throw
    throw std::runtime_error("");
  }
}

ApriltagMap loadApriltagMapYaml(const std::string& filename) {
  const auto file = YAML::LoadFile(filename);
  const auto tag_family = file["tag_family"].as<std::string>();
  const auto tag_size = file["tag_size"].as<double>();
  ApriltagMap apriltag_map(tag_family, tag_size);

  const YAML::Node& tags = file["tags"];
  for (size_t i = 0; i < tags.size(); ++i) {
    const YAML::Node& tag = tags[i];
    const auto id = tag["id"].as<int>();
    const long code = tag["code"].as<long>();
    const YAML::Node& orientation = tag["orientation"];
    const YAML::Node& position = tag["position"];

    ApriltagMap::Tag3D tag_3d(id);
    {
      const auto w = orientation["w"].as<double>();
      const auto x = orientation["x"].as<double>();
      const auto y = orientation["y"].as<double>();
      const auto z = orientation["z"].as<double>();
      tag_3d.set_q({w, x, y, z});
    }

    {
      const auto x = position["x"].as<double>();
      const auto y = position["y"].as<double>();
      const auto z = position["z"].as<double>();
      tag_3d.set_p({x, y, z});
    }

    apriltag_map.addTag(tag_3d);
    std::cout << "add tag: " << id << "/" << code << std::endl;
  }
  return apriltag_map;
}

}  // namespace apriltag_ros
