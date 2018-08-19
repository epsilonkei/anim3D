#ifndef GRID_CELL_HPP
#define GRID_CELL_HPP

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "grid_point.hpp"

class grid_cell
{
public:
  std::vector<grid_cell*> neighbours;
  std::vector<grid_point*> points;
  int i, j, k;

  grid_cell()
    : i(0), j(0), k(0)
  {}
  ~ grid_cell() {}

  void initialize(int ii, int jj, int kk) {
    i = ii; j = jj; k = kk;
  }

  void reset() {
    for (uint i=0; i<points.size(); i++) {
      points[i]->isInGridCell = false;
    }
    points.clear();
    i = 0; j = 0; k = 0;
  }

  void insertGridPoint(grid_point *gp) {
    gp->i = i; gp->j = j; gp->k = k;
    gp->isInGridCell = true;
    points.push_back(gp);
  }

  void removeGridPoint(grid_point *gp) {
    for (uint i=0; i<points.size(); i++) {
      if (points[i]->id == gp->id) {
        gp->isInGridCell = false;
        points.erase(points.begin() + i);
        return;
      }
    }
  }

  bool isEmpty() {
    return points.size() == 0;
  }
};

#endif // GRID_CELL_HPP
