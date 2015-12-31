#ifndef APRILTAGS_UNIONFINDSIMPLE_H_
#define APRILTAGS_UNIONFINDSIMPLE_H_

#include <vector>

namespace AprilTags {

//! Implementation of disjoint set data structure using the union-find algorithm
class UnionFind {
  //! Identifies parent ids and sizes.
  struct Data {
    int id;
    int size;
  };

 public:
  explicit UnionFind(int max_ids);

  int GetSetSize(int id) { return data_[GetRepresentative(id)].size; }

  int GetRepresentative(int id);

  //! Returns the id of the merged node.
  int ConnectNodes(int id0, int id1);

 private:
  std::vector<Data> data_;
};

}  // namespace AprilTags

#endif  // APRILTAGS_UNIONFINDSIMPLE_H_
