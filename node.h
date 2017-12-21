#ifndef NODE_H
#define NODE_H

#include "gl_const.h"

#include <limits>
#include <list>
#include <iostream>
#include <cmath>


struct Node {

    Node*   parent;

    int     i, j;
    int     radius;
    float   F;
    float   g;

    double angle;

    Node() : i(-1), j(-1), F(std::numeric_limits<float>::infinity()), g(std::numeric_limits<float>::infinity()),
             parent(nullptr), radius(CN_PTD_D), angle(0) {}

    Node(int x, int y, float g_=std::numeric_limits<float>::infinity(), float h_=std::numeric_limits<float>::infinity(),
         float radius_=CN_PTD_D, Node *parent_=nullptr, float cweightdist_=0, double ang_=0) :
        i(x), j(y), g(g_), radius(radius_), parent(parent_), angle(ang_) {
        if (parent) {
            F = g + h_ + cweightdist_ * fabs(ang_ - parent->angle);
        } else {
            F = g + h_;
        }
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
        angle = other.angle;
        radius = other.radius;
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
