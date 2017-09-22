#include "AprilTags/DisjointSets.h"
#include <iostream>

namespace AprilTags {

DisjointSets::DisjointSets(size_t n) : parent_(n), size_(n, 1) {
  for (size_t i = 0; i < n; ++i) {
    // everyone is their own cluster of size 1
    parent_[i] = i;
  }
}

size_t DisjointSets::Find(size_t id) {
  // terminal case: a node is its own parent
  const auto parent = parent_[id];
  if (parent == id) return id;

  // otherwise, recurse...
  auto root = Find(parent);

  // short circuit the path
  parent_[id] = root;

  return root;
}

size_t DisjointSets::Union(size_t id0, size_t id1) {
  const auto root0 = Find(id0);
  const auto root1 = Find(id1);

  if (root0 == root1) return root0;

  const auto sz0 = size_[root0];
  const auto sz1 = size_[root1];

  if (sz0 > sz1) {
    parent_[root1] = root0;
    size_[root0] += sz1;
    return root0;
  } else {
    parent_[root0] = root1;
    size_[root1] += sz0;
    return root1;
  }
}

size_t DisjointSets::GetSetSize(size_t id) { return size_[Find(id)]; }

}  // namespace AprilTags
