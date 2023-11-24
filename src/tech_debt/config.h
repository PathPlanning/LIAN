#ifndef CONFIG_H
#define CONFIG_H

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

class Config {

private:
    int N;
    float *searchParams;

public:
    Config();
    Config(int numParams, float *paramArray);
    ~Config();

    float getParamValue(int i) const;

    bool getConfig(const char* FileName);
};

#endif
