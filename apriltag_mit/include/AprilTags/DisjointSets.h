#ifndef APRILTAGS_DISJOINTSETS_H_
#define APRILTAGS_DISJOINTSETS_H_

#include <cstddef>
#include <vector>

namespace AprilTags {

class DisjointSets {
 public:
  explicit DisjointSets(size_t n);

  size_t GetSetSize(size_t id);

  size_t Find(size_t id);

  //! Returns the id of the merged node.
  size_t Union(size_t id0, size_t id1);

 private:
  std::vector<size_t> parent_;
  std::vector<size_t> size_;
};

}  // namespace AprilTags

#endif  // APRILTAGS_DISJOINTSETS_H_
