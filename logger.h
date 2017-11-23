#ifndef LOGGER_H
#define LOGGER_H

#include "map.h"
#include "node.h"
#include "openlist.h"

#include <list>
#include <vector>
#include <unordered_map>


class Logger {

public:
    float loglevel;

public:
    Logger() : loglevel(-1) {}
    virtual ~Logger() {}
    virtual bool getLog(const char* FileName) = 0;
    virtual void saveLog() = 0;
    virtual void writeToLogMap(const Map &map,const std::list<Node> &path) = 0;
    virtual void writeToLogOpenClose(const OpenList &open, const std::unordered_multimap<int, Node>& close, const int size) = 0;
    virtual void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles) = 0;
    virtual void writeToLogHpLevel(const std::list<Node> &path) = 0;
    virtual void writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, float length_scaled,
                                   long double time, float max_angle, float accum_angle, int sections) = 0;};

#endif
