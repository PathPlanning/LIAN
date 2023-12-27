#ifndef CONFIG_H
#define CONFIG_H

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

struct SearchParams {
    float angleLimit;
    int distance;
    float weight;
    unsigned int stepLimit;
    float curvatureHeuristicWeight;
    bool postsmoother;
    float decreaseDistanceFactor;
    int distanceMin;
    double pivotRadius;
    int numOfParentsToIncreaseRadius;
    int logLevel;
};

class Config {
public:
    Config(const char* FileName);

    const SearchParams& params() const;
private:
    SearchParams params_;
};

#endif
