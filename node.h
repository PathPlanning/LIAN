#ifndef NODE_H
#define NODE_H

#include "gl_const.h"

#include <limits>
#include <list>
#include <iostream>


struct Node {

    Node*   parent;

    bool    pathToParent;
    int     i, j;
    int     radius;
    float   F;
    float   g;
    float   c; // curvature euristic component

    Node() : i(-1), j(-1), F(std::numeric_limits<float>::infinity()), g(std::numeric_limits<float>::infinity()),
    c(-1), parent(nullptr), pathToParent(false), radius(CN_PTD_D) {}

    Node(int x, int y, float g_=std::numeric_limits<float>::infinity(), float h_=std::numeric_limits<float>::infinity(),
         float radius_=CN_PTD_D, Node *parent_=nullptr, float c_=-1, float cweightdist_=0) :
        i(x), j(y), g(g_), radius(radius_), c(c_), parent(parent_) {
        F = g + h_ + cweightdist_ * c;
    }

    ~Node() {
        parent = nullptr;
    }

    inline Node& operator=(const Node& other) {
        i = other.i;
        j = other.j;
        F = other.F;
        g = other.g;
        parent = other.parent;
        pathToParent = other.pathToParent;
        radius = other.radius;
        c = other.c;
        return *this;
    }

    inline bool operator==(const Node& p) const {
            return i == p.i && j == p.j && parent->i == p.parent->i && parent->j == p.parent->j;
    }

    inline bool operator!=(const Node& p) const {
            return !(*this == p);
    }

    int convolution(int width) const {
            return i * width + j;
    }
};

#endif
