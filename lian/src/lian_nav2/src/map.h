#ifndef MAP_H
#define MAP_H

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

this->declare_parameter("traversable_threshold", 50.0);
this->get_parameter("traversable_threshold", traversable_threshold);

class Map {

private:
    int ** Grid;
    int height, width;
    double CellSize;

public:
    Map();
    ~Map();
    bool getMap(OccupancyGrid occupancyGrid_msg);

    bool CellIsTraversable (int curr_i, int curr_j) const;
    bool CellOnGrid (int curr_i, int curr_j) const;
    bool CellIsObstacle(int curr_i, int curr_j) const;

    int* operator [] (int i);
    const int* operator [] (int i) const;

    int getWidth() const;
    int getHeight() const;
    double getCellSize() const;

};

#endif
