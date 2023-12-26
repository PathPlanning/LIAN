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
    unsigned int steplimit;
    float curvatureHeuristicWeight;
    bool postsmoother;
    float decreaseDistanceFactor;
    int distanceMin;
    double pivotRadius;
    int numOfParentsToIncreaseRadius;
    float logLevel;
};

class Config {
public:
    Config() {}

    const SearchParams& params() const;

    bool getConfig(const char* FileName);
private:
    SearchParams params_;
};

#endif
