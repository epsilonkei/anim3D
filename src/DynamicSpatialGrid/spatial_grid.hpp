#ifndef SPATIAL_GRID_HPP
#define SPATIAL_GRID_HPP

#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Dense>
#include <unordered_map>
#include <GL/glu.h>
#include <cmath>
#include "cell_hash.hpp"
// #include "utils.hpp"
#include "grid_cell.hpp"

class spatial_grid {
public:
  double size;
  int currentGridPointID;
  std::vector<grid_point*> points;
  std::unordered_map<int,grid_point*> gridPointsByID;
  std::vector<grid_cell*> freeCells;
  int numInitialFreeCells;
  cell_hash cellHashTable;
  bool isCellRemoved = false;

  spatial_grid() {
    currentGridPointID = 0;
    numInitialFreeCells = 10000;
    initializeFreeCells();
  }
  ~ spatial_grid() {}

  void init(double _size) {
    size = _size;
  }

  int insertPoint(Eigen::Vector3d p) {
    grid_point *point = new grid_point();
    point->position = p;
    point->id = generateUniqueGridPointID();
    point->isMarkedForRemoval = false;
    points.push_back(point);
    std::pair<int,grid_point*> pair(point->id, point);
    gridPointsByID.insert(pair);
    insertGridPointIntoGrid(point);
    return point->id;
  }

  void movePoint(int id, Eigen::Vector3d newPos) {
    if (gridPointsByID.find(id) == gridPointsByID.end()) {
      return;
    }
    grid_point *point = gridPointsByID[id];
    int i = point->i;
    int j = point->j;
    int k = point->k;
    Eigen::Vector3d trans = newPos - point->position;
    point->tx += trans[0];
    point->ty += trans[1];
    point->tz += trans[2];
    point->position = newPos;
    // point has moved to new cell
    if (point->tx >= size || point->ty >= size || point->tz >= size ||
        point->tx < 0 || point->ty < 0 || point->tz < 0) {
      int nexti, nextj, nextk;
      positionToIJK(point->position, &nexti, &nextj, &nextk);
      // remove grid point from old cell
      grid_cell *oldCell = cellHashTable.getGridCell(i, j, k);
      oldCell->removeGridPoint(point);
      // remove cell from hash if empty
      if (oldCell->isEmpty()) {
        cellHashTable.removeGridCell(oldCell);
        oldCell->reset();
        freeCells.push_back(oldCell);
      }
      // insert into new cell
      bool isCellInTable = false;
      grid_cell *cell = cellHashTable.findGridCell(nexti, nextj, nextk, &isCellInTable);
      if (isCellInTable) {
        cell->insertGridPoint(point);
      } else {
        grid_cell *cell = getNewGridCell(nexti, nextj, nextk);
        cell->insertGridPoint(point);
        cellHashTable.insertGridCell(cell);
      }
      updateGridPointCellOffset(point, nexti, nextj, nextk);
    }
  }

  void removePoint(int id) {
    if (gridPointsByID.find(id) == gridPointsByID.end()) {
      return;
    }
    isCellRemoved = true;
    grid_point *point = gridPointsByID[id];
    gridPointsByID.erase(id);
    int i = point->i;
    int j = point->j;
    int k = point->k;
    point->isMarkedForRemoval = true;
    bool isCellInTable = false;
    grid_cell *cell = cellHashTable.findGridCell(i, j, k, &isCellInTable);
    if (!isCellInTable) {
      return;
    }
    cell->removeGridPoint(point);
    if (cell->isEmpty()) {
      cellHashTable.removeGridCell(cell);
      cell->reset();
      freeCells.push_back(cell);
    }
  }
  std::vector<Eigen::Vector3d> getObjectsInRadiusOfPoint(int ref, double r) {
    std::vector<Eigen::Vector3d> objects;
    if (gridPointsByID.find(ref) == gridPointsByID.end()) {
      return objects;
    }
    grid_point *p = gridPointsByID[ref];
    double tx = p->tx;
    double ty = p->ty;
    double tz = p->tz;
    int i, j, k;
    positionToIJK(p->position, &i, &j, &k);
    double inv = 1/size;
    double rsq = r*r;
    int imin = i - fmax(0, ceil((r-tx)*inv));
    int jmin = j - fmax(0, ceil((r-ty)*inv));
    int kmin = k - fmax(0, ceil((r-tz)*inv));
    int imax = i + fmax(0, ceil((r-size+tx)*inv));
    int jmax = j + fmax(0, ceil((r-size+ty)*inv));
    int kmax = k + fmax(0, ceil((r-size+tz)*inv));
    grid_cell *cell;
    grid_point *gp;
    Eigen::Vector3d v;
    std::vector<grid_point*> points;
    for (int ii=imin; ii<=imax; ii++) {
      for (int jj=jmin; jj<=jmax; jj++) {
        for (int kk=kmin; kk<=kmax; kk++) {
          bool isInHash = false;
          cell = cellHashTable.findGridCell(ii, jj, kk, &isInHash);
          if (isInHash) {
            points = cell->points;
            for (int idx=0; idx<(int)points.size(); idx++) {
              gp = points[idx];
              if (gp->id != ref) {
                v = p->position - gp->position;
                if (v.squaredNorm() < rsq) {
                  objects.push_back(gp->position);
                }
              }
            }
          }
        }
      }
    }
    return objects;
  }

  std::vector<int> getIDsInRadiusOfPoint(int ref, double r) {
    if (gridPointsByID.find(ref) == gridPointsByID.end()) {
      std::vector<int> objects;
      return objects;
    }
    grid_point *p = gridPointsByID[ref];
    double tx = p->tx;
    double ty = p->ty;
    double tz = p->tz;
    int i, j, k;
    positionToIJK(p->position, &i, &j, &k);
    double inv = 1/size;
    double rsq = r*r;
    int imin = i - fmax(0, ceil((r-tx)*inv));
    int jmin = j - fmax(0, ceil((r-ty)*inv));
    int kmin = k - fmax(0, ceil((r-tz)*inv));
    int imax = i + fmax(0, ceil((r-size+tx)*inv));
    int jmax = j + fmax(0, ceil((r-size+ty)*inv));
    int kmax = k + fmax(0, ceil((r-size+tz)*inv));
    if (imax - imin <= 3 and imax - imin >= 1) {
      return fastIDNeighbourSearch(ref, r, p);
    }
    std::vector<int> objects;
    grid_point *gp;
    grid_cell *cell;
    Eigen::Vector3d v;
    std::vector<grid_point*> points;
    for (int ii=imin; ii<=imax; ii++) {
      for (int jj=jmin; jj<=jmax; jj++) {
        for (int kk=kmin; kk<=kmax; kk++) {
          bool isInHash = false;
          cell = cellHashTable.findGridCell(ii, jj, kk, &isInHash);
          if (isInHash) {
            points = cell->points;
            for (int idx=0; idx<(int)points.size(); idx++) {
              gp = points[idx];
              if (gp->id != ref) {
                v = p->position - gp->position;
                if (v.squaredNorm() < rsq) {
                  objects.push_back(gp->id);
                }
              }
            }
          }
        }
      }
    }
    return objects;
  }

  void update() {
    removeGridPointsMarkedForRemoval();
    // update each cell's cell neighbours
    std::vector<grid_cell*> cells;
    cellHashTable.getGridCells(&cells);
    grid_cell* cell;
    grid_cell* gc;
    for (uint idx=0; idx<cells.size(); idx++) {
      cell = cells[idx];
      cell->neighbours.clear();
      int ii = cell->i;
      int jj = cell->j;
      int kk = cell->k;
      for (int k=kk-1; k<=kk+1; k++) {
        for (int j=jj-1; j<=jj+1; j++) {
          for (int i=ii-1; i<=ii+1; i++) {
            if (!(i==ii && j==jj && k==kk)) {
              bool isInTable = false;
              gc = cellHashTable.findGridCell(i, j, k, &isInTable);
              if (isInTable) {
                cell->neighbours.push_back(gc);
              }
            }
          }
        }
      }
    }
  }

  // void draw() {
  //   if (points.size() == 0) { return; }
  //   glColor3f(1.0, 0.4, 0.0);
  //   glPointSize(6.0);
  //   glBegin(GL_POINTS);
  //   for (int i=0; i<(int)points.size(); i++) {
  //     Eigen::Vector3d p = points[i]->position;
  //     glVertex3f(p.x, p.y, p.z);
  //   }
  //   glEnd();
  //   std::vector<grid_cell*> cells;
  //   cellHashTable.getGridCells(&cells);
  //   glLineWidth(1.0);
  //   glColor4f(0.0, 0.0, 1.0, 0.4);
  //   for (int i=0; i<(int)cells.size(); i++) {
  //     grid_cell *c = cells[i];
  //     Eigen::Vector3d pos = IJKToPosition(c->i, c->j, c->k);
  //     pos = pos + Eigen::Vector3d::Ones() * 0.5 * size;
  //     utils::drawWireframeCube(pos, size);
  //   }
  // }

  int generateUniqueGridPointID() {
    int id = currentGridPointID;
    currentGridPointID++;
    return id;
  }

  void initializeFreeCells() {
    for (int i=0; i<numInitialFreeCells; i++) {
      grid_cell *cell = new grid_cell();
      freeCells.push_back(cell);
    }
  }

  void insertGridPointIntoGrid(grid_point *p) {
    int i, j, k;
    positionToIJK(p->position, &i, &j, &k);
    bool isCellInTable = false;
    grid_cell *cell = cellHashTable.findGridCell(i, j, k, &isCellInTable);
    if (isCellInTable) {
      cell->insertGridPoint(p);
    } else {
      cell = getNewGridCell(i, j ,k);
      cell->insertGridPoint(p);
      cellHashTable.insertGridCell(cell);
    }
    // offset used for updating position
    updateGridPointCellOffset(p, i, j, k);
  }

  void positionToIJK(Eigen::Vector3d p, int *i, int *j, int *k) {
    double inv = 1 / size;
    *i = ceil(p[0]*inv)-1;
    *j = ceil(p[1]*inv)-1;
    *k = ceil(p[2]*inv)-1;
    double eps = 0.00000000001;
    if (fabs(fmod(p[0], size)) < eps) {
      *i = *i + 1;
    }
    if (fabs(fmod(p[1], size)) < eps) {
      *j = *j + 1;
    }
    if (fabs(fmod(p[2], size)) < eps) {
      *k = *k + 1;
    }
  }

  Eigen::Vector3d IJKToPosition(int i, int j, int k) {
    Eigen::Vector3d ret(i*size, j*size, k*size);
    return ret;
  }

  grid_cell* getNewGridCell(int i, int j, int k) {
    if (freeCells.size() == 0) {
      int n = 200;
      for (int i=0; i<n; i++) {
        freeCells.push_back(new grid_cell());
      }
    }
    grid_cell *cell = freeCells.back();
    freeCells.pop_back();
    cell->initialize(i, j, k);
    return cell;
  }

  void updateGridPointCellOffset(grid_point *gp, int i, int j, int k) {
    Eigen::Vector3d cp = IJKToPosition(i, j, k);
    gp->tx = gp->position[0] - cp[0];
    gp->ty = gp->position[1] - cp[1];
    gp->tz = gp->position[2] - cp[2];
  }

  std::vector<int> fastIDNeighbourSearch(int ref, double r, grid_point *p) {
    std::vector<int> objects;
    bool isInHash = false;
    grid_cell *cell = cellHashTable.findGridCell(p->i, p->j, p->k, &isInHash);
    if (!isInHash) {
      return objects;
    }
    std::vector<grid_point*> points = cell->points;
    grid_point *gp;
    Eigen::Vector3d v;
    double rsq = r*r;
    for (uint i=0; i<points.size(); i++) {
      gp = points[i];
      if (gp->id != ref) {
        v = p->position - gp->position;
        if (v.squaredNorm() < rsq) {
          objects.push_back(gp->id);
        }
      }
    }
    std::vector<grid_cell*> neighbours = cell->neighbours;
    for (uint i=0; i<neighbours.size(); i++) {
      points = neighbours[i]->points;
      for (uint j=0; j<points.size(); j++) {
        gp = points[j];
        v = p->position - gp->position;
        if (v.squaredNorm() < rsq) {
          objects.push_back(gp->id);
        }
      }
    }
    return objects;
  }

  void removeGridPointsMarkedForRemoval() {
    if (points.size() == 0 || !isCellRemoved) {
      return;
    }
    grid_point *p;
    for (int i=(int)points.size()-1; i>=0; i--) {
      if (points[i]->isMarkedForRemoval) {
        p = points[i];
        points.erase(points.begin() + i);
        delete p;
      }
    }
    isCellRemoved = false;
  }
};

#endif // SPATIAL_GRID_HPP
