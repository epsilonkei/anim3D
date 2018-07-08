#ifndef CELL_HASH_HPP
#define CELL_HASH_HPP

#include <unordered_map>
#include <vector>
#include "grid_cell.hpp"

class cell_hash
{
public:
  long maxNumHashValues;
  std::unordered_map<long, std::vector<grid_cell*>> cellMap;

  cell_hash()
    : maxNumHashValues(10000)
  {}
  ~ cell_hash() {}

  bool isGridCellInHash(int i, int j, int k) {
    long h = computeHash(i, j, k);
    if (cellMap.find(h) == cellMap.end()) {
        return false;
    }
    grid_cell *c;
    std::vector<grid_cell*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        c = chain[idx];
        if (c->i == i && c->j == j and c->k == k) {
            return true;
        }
    }
    return false;
  }

  void insertGridCell(grid_cell *cell) {
    long h = computeHash(cell->i, cell->j, cell->k);
    if (cellMap.find(h) == cellMap.end()) {
        std::vector<grid_cell*> newChain;
        std::pair<long,std::vector<grid_cell*>> pair(h, newChain);
        cellMap.insert(pair);
    }
    cellMap[h].push_back(cell);
  }

  void removeGridCell(grid_cell *cell) {
    int i = cell->i;
    int j = cell->j;
    int k = cell->k;
    long h = computeHash(i, j, k);
    if (cellMap.find(h) == cellMap.end()) {
      std::cerr << "cant find cell" << i << j << k << h << std::endl;
        return;
    }
    // remove from hash chain
    bool isRemoved = false;
    std::vector<grid_cell*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        grid_cell *c = (cellMap[h])[idx];
        if (c->i == i && c->j == j && c->k == k) {
            cellMap[h].erase(cellMap[h].begin() + idx);
            isRemoved = true;
            break;
        }
    }
    if (!isRemoved) {
      std::cerr << "Could not find/remove gridcell" << i << j << k << std::endl;
    }
    // remove chain from map if empty
    if (chain.size() == 0) {
        cellMap.erase(h);
    }
  }

  grid_cell* getGridCell(int i, int j, int k) {
    long h = computeHash(i, j, k);
    grid_cell *c;
    std::vector<grid_cell*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        c = chain[idx];
        if (c->i == i && c->j == j and c->k == k) {
            return c;
        }
    }
    return c;
  }

  grid_cell* findGridCell(int i, int j, int k, bool *isGridCellFound) {
    long h = computeHash(i, j, k);
    grid_cell *c;
    std::vector<grid_cell*> chain = cellMap[h];
    for (int idx=0; idx<(int)chain.size(); idx++) {
        c = chain[idx];
        if (c->i == i && c->j == j and c->k == k) {
            *isGridCellFound = true;
            return c;
        }
    }
    *isGridCellFound = false;
    return c;
  }

  void getGridCells(std::vector<grid_cell*> *cells) {
    for (std::pair<int, std::vector<grid_cell*>> pair: cellMap) {
        for (int i=0; i < (int)pair.second.size(); i++) {
            cells->push_back(pair.second[i]);
        }
    }
  }

  inline long computeHash(int i, int j, int k) {
    return (abs(541*(long)i + 79*(long)j + 31*(long)k) % maxNumHashValues);
  }

};

#endif //CELL_HASH_HPP
