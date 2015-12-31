#include "AprilTags/UnionFind.h"
#include <iostream>

namespace AprilTags {

UnionFind::UnionFind(int max_ids) : data_(max_ids) {
  for (size_t i = 0; i < data_.size(); ++i) {
    // everyone is their own cluster of size 1
    data_[i].id = i;
    data_[i].size = 1;
  }
}

int UnionFind::GetRepresentative(int id) {
  // terminal case: a node is its own parent
  if (data_[id].id == id) return id;

  // otherwise, recurse...
  int root = GetRepresentative(data_[id].id);

  // short circuit the path
  data_[id].id = root;

  return root;
}

int UnionFind::ConnectNodes(int id0, int id1) {
  int root0 = GetRepresentative(id0);
  int root1 = GetRepresentative(id1);

  if (root0 == root1) return root0;

  int asz = data_[root0].size;
  int bsz = data_[root1].size;

  if (asz > bsz) {
    data_[root1].id = root0;
    data_[root0].size += bsz;
    return root0;
  } else {
    data_[root0].id = root1;
    data_[root1].size += asz;
    return root1;
  }
}

}  // namespace
