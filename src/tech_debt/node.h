#ifndef NODE_H
#define NODE_H

#include "gl_const.h"

#include <limits>
#include <list>
#include <iostream>
#include <cmath>

struct circleNode
{
    int i, j;
    double heading;
    double cost;

    circleNode(): i(-1), j(-1), heading(-1), cost(0) {}
    circleNode(int i, int j, double cost): i(i), j(j), heading(-1), cost(cost) {}
    circleNode(int i, int j, double heading, double cost): i(i), j(j), heading(heading), cost(cost) {}
};

struct Node {
private:
    static constexpr int defaultRadius = 4;
public:
    Node*   parent;

    int     i, j;
    int   radius;
    float   F;
    float   g;

    double angle;

    Node() : parent(nullptr), i(-1), j(-1), radius(defaultRadius),
             F(std::numeric_limits<float>::infinity()), g(std::numeric_limits<float>::infinity()), angle(0) {}

    Node(int x, int y, float g_=std::numeric_limits<float>::infinity(), float h_=std::numeric_limits<float>::infinity(),
         int radius_= defaultRadius, Node *parent_=nullptr, float cweightdist_=0, double ang_=0) :
        parent(parent_), i(x), j(y), radius(radius_), g(g_), angle(ang_) {
        if (parent) {
            F = g + h_ + cweightdist_ * (float)fabs(ang_ - parent->angle);
        } else {
            F = g + h_;
        }
    }

    ~Node() {
        parent = nullptr;
    }

    inline Node& operator=(const Node& other) = default;

    inline bool operator==(const Node& other) const {
        return i == other.i && j == other.j;
    }

    inline bool operator!=(const Node& other) const {
            return !(*this == other);
    }

    // Warning: Order in which equal nodes would be is undefined and platform-specific.
    // Result in different platforms may differ.
    inline bool operator<(const Node& other) const {
        return F > other.F || (F == other.F && g < other.g);
    }
};

template<>
struct std::hash<Node> {
    std::size_t operator()(const Node& node) const
    {
        return (size_t)node.i << 32 | (size_t)node.j;
    }
};

inline bool areFromSameSource(const Node& lhs, const Node& rhs) {
    return ((!lhs.parent && !rhs.parent)
            || (lhs.parent && rhs.parent
                && lhs.parent->i == rhs.parent->i && lhs.parent->j == rhs.parent->j));
}

#endif
