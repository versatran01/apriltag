#ifndef APRILTAGS_GRIDDER_H_
#define APRILTAGS_GRIDDER_H_

#include <algorithm>
#include <iterator>
#include <vector>

#include "AprilTags/Segment.h"

namespace AprilTags {

//! A lookup table in 2D for implementing nearest neighbor.
template <class T>
class Gridder {
 private:
  Gridder(const Gridder&);             //!< don't call
  Gridder& operator=(const Gridder&);  //!< don't call

  struct Cell {
    T* object;
    Cell* next;

    Cell() : object(nullptr), next(nullptr) {}

    Cell(const Cell& c) : object(c.object), next(c.next) {}

    // Destructor
    ~Cell() { delete next; }

    Cell& operator=(const Cell& other) {
      if (this == &other) return *this;

      object = other.object;
      next = other.next;
      return *this;
    }
  };

  //! Initializes Gridder constructor
  void gridderInit(float x0Arg, float y0Arg, float x1Arg, float y1Arg,
                   float ppCell) {
    width_ = (int)((x1Arg - x0Arg) / ppCell + 1);
    height_ = (int)((y1Arg - y0Arg) / ppCell + 1);

    x1_ = x0Arg + ppCell * width_;
    y1_ = y0Arg + ppCell * height_;
    cells_ = std::vector<std::vector<Cell*>>(
        height_, std::vector<Cell*>(width_, (Cell*)nullptr));
  }

  float x0_, y0_, x1_, y1_;
  int width_, height_;
  float pixels_per_cell_;
  std::vector<std::vector<Cell*>> cells_;

 public:
  Gridder(float x0Arg, float y0Arg, float x1Arg, float y1Arg, float ppCell)
      : x0_(x0Arg),
        y0_(y0Arg),
        x1_(),
        y1_(),
        width_(),
        height_(),
        pixels_per_cell_(ppCell),
        cells_() {
    gridderInit(x0Arg, y0Arg, x1Arg, y1Arg, ppCell);
  }

  // Destructor
  ~Gridder() {
    for (unsigned int i = 0; i < cells_.size(); i++) {
      for (unsigned int j = 0; j < cells_[i].size(); j++) {
        delete cells_[i][j];
      }
    }
  }

  void Add(float x, float y, T* object) {
    int ix = (int)((x - x0_) / pixels_per_cell_);
    int iy = (int)((y - y0_) / pixels_per_cell_);

    if (ix >= 0 && iy >= 0 && ix < width_ && iy < height_) {
      Cell* c = new Cell;
      c->object = object;
      c->next = cells_[iy][ix];
      cells_[iy][ix] = c;
    }
  }

  // iterator begin();
  // iterator end();

  //! Iterator for Segment class.
  class Iterator {
   public:
    Iterator(Gridder* grid, float x, float y, float range)
        : outer(grid), ix0(), ix1(), iy0(), iy1(), ix(), iy(), c(NULL) {
      iteratorInit(x, y, range);
    }

    Iterator(const Iterator& it)
        : outer(it.outer),
          ix0(it.ix0),
          ix1(it.ix1),
          iy0(it.iy0),
          iy1(it.iy1),
          ix(it.ix),
          iy(it.iy),
          c(it.c) {}

    Iterator& operator=(const Iterator& it) {
      outer = it.outer;
      ix0 = it.ix0;
      ix1 = it.ix1;
      iy0 = it.iy0;
      iy1 = it.iy1;
      ix = it.ix;
      iy = it.iy;
      c = it.c;
    }

    bool hasNext() {
      if (c == nullptr) findNext();
      return (c != nullptr);
    }

    T& next() {
      T* thisObj = c->object;
      findNext();
      return *thisObj;  // return Segment
    }

   private:
    void findNext() {
      if (c != nullptr) c = c->next;
      if (c != nullptr) return;

      ix++;
      while (true) {
        if (ix > ix1) {
          iy++;
          ix = ix0;
        }
        if (iy > iy1) break;

        c = outer->cells_[iy][ix];

        if (c != nullptr) break;
        ix++;
      }
    }

    //! Initializes Iterator constructor
    void iteratorInit(float x, float y, float range) {
      ix0 = (int)((x - range - outer->x0_) / outer->pixels_per_cell_);
      iy0 = (int)((y - range - outer->y0_) / outer->pixels_per_cell_);

      ix1 = (int)((x + range - outer->x0_) / outer->pixels_per_cell_);
      iy1 = (int)((y + range - outer->y0_) / outer->pixels_per_cell_);

      ix0 = std::max(0, ix0);
      ix0 = std::min(outer->width_ - 1, ix0);

      ix1 = std::max(0, ix1);
      ix1 = std::min(outer->width_ - 1, ix1);

      iy0 = std::max(0, iy0);
      iy0 = std::min(outer->height_ - 1, iy0);

      iy1 = std::max(0, iy1);
      iy1 = std::min(outer->height_ - 1, iy1);

      ix = ix0;
      iy = iy0;

      c = outer->cells_[iy][ix];
    }

    Gridder* outer;
    int ix0, ix1, iy0, iy1;
    int ix, iy;
    Cell* c;
  };

  typedef Iterator iterator;
  iterator find(float x, float y, float range) {
    return Iterator(this, x, y, range);
  }
};

}  // namespace AprilTags

#endif  // APRILTAGS_GRIDDER_H_
