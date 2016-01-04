#ifndef APRILTAGS_DISJOINTSETS_H_
#define APRILTAGS_DISJOINTSETS_H_

#include <vector>

namespace AprilTags {

//! Implementation of disjoint set data structure using the union-find algorithm
class DisjointSets {
  //! Identifies parent ids and sizes.
  struct Data {
    int id;
    int size;
  };

 public:
  explicit DisjointSets(int max_ids);

  int GetSetSize(int id) { return data_[GetRepresentative(id)].size; }

  int GetRepresentative(int id);

  //! Returns the id of the merged node.
  int ConnectNodes(int id0, int id1);

 private:
  std::vector<Data> data_;
};

/**
 * @brief The Element struct
 * Element in our disjoint sets
 */
struct Element {
  int pid, size;
  float mmin, mmax, tmin, tmax;
};

namespace exp {

class DisjointSets {
 public:
  DisjointSets(int n);

 private:
  std::vector<int> rank_;
  std::vector<int> parent_;
};
}

}  // namespace AprilTags

#endif  // APRILTAGS_DISJOINTSETS_H_
