#ifndef GRID_POINT_HPP
#define GRID_POINT_HPP

#include <vector>
#include <Eigen/Dense>

struct grid_point {
  Eigen::Vector3d position;
  int id;
  double tx, ty, tz;
  int i, j, k;
  bool isInGridCell = false;
  bool isMarkedForRemoval = false;
};

#endif //GRID_POINT_HPP
