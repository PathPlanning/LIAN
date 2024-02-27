#ifndef SEARCH_RESULT_H
#define SEARCH_RESULT_H

#include "node.h"
#include <vector>

struct SearchResult {

    bool pathFound{};
    float pathLength{};
    std::list<Node> hpPath;
    std::list<Node> lpPath;
    unsigned int nodesCreated{};
    unsigned int numberOfSteps{};
    std::vector<float> angles;
    float accumAngle{};
    double time{};
    float maxAngle{};
    int sections{};
};

#endif // SEARCH_RESULT_H
