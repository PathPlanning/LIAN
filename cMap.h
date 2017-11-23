#ifndef CMAP_H
#define CMAP_H

#include<iostream>
#include"tinyxml.h"
#include"tinystr.h"
#include "gl_const.h"

class cMap
{
public:
    int **Grid;
    int height, width;
    int start_i, start_j;
    int goal_i, goal_j;

public:
    cMap();
    ~cMap();
    bool getMap(const char* FileName);
};

#endif
