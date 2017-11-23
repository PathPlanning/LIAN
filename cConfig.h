#ifndef CCONFIG_H
#define CCONFIG_H

#include<iostream>
#include"tinyxml.h"
#include"tinystr.h"
#include<string>
#include"gl_const.h"


class cConfig
{
public:
    float *searchParams;
    int N;
public:
    cConfig();
    cConfig(int numParams, float *paramArray);
    ~cConfig();

    bool getConfig(const char* FileName);
};

#endif
