#ifndef SNODE_H
#define SNODE_H
#include <list>
#include "gl_const.h"
#include <iostream>
struct Node
{
    int     i, j;
    float   F;
    float     g;
    Node*   Parent;
    bool    pathToParent;
    int     radius;

    float   c;// curvature euristic component
              // компонент эвристики, которая учитывает "искревленность" построенного пути:
              // чем ближе путь к прямой, тем лучше

    Node()
    {
        i = -1;
        j = -1;
        F = -1;
        g = -1;
        c = -1;
        Parent = NULL;
        pathToParent = false;
        radius = CN_PTD_D;
    }

    Node(int x, int y, float f, float G, float C)
    {
        i = x;
        j = y;
        F = f;
        g = G;
        c = C;
        Parent = NULL;
        pathToParent = false;
        radius = CN_PTD_D;
    }

    ~Node()
    {
        Parent = NULL;
    }
};

#endif
