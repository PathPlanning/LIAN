#ifndef MAP_H
#define MAP_H

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>


class Map {

private:
    int ** Grid;
    int height, width;
    double CellSize;

public:
    Map();
    ~Map();
    bool getMap(const char* FileName);

    bool CellIsTraversable (int curr_i, int curr_j) const;
    bool CellOnGrid (int curr_i, int curr_j) const;
    bool CellIsObstacle(int curr_i, int curr_j) const;

    int* operator [] (int i);
    const int* operator [] (int i) const;

    int getWidth() const;
    int getHeight() const;
    double getCellSize() const;


    int start_i, start_j;
    int goal_i, goal_j;
};

#endif
