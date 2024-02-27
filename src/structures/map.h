#ifndef MAP_H
#define MAP_H

#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <vector>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>


class Map {
private:
    std::vector<std::vector<int>> grid_;
    int height_, width_;
    double cellSize_;

public:
    Map(const char* FileName);

    bool CellIsTraversable(int curr_i, int curr_j) const;
    bool CellOnGrid(int curr_i, int curr_j) const;
    bool CellIsObstacle(int curr_i, int curr_j) const;

    std::vector<int>& operator[](int i);
    const std::vector<int>& operator[](int i) const;

    int getWidth() const;
    int getHeight() const;
    double getCellSize() const;

    int start_i, start_j;
    int goal_i, goal_j;
};

#endif
