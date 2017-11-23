#ifndef CLOGGER_H
#define CLOGGER_H

#include"cMap.h"
#include"cList.h"
#include <vector>
#include <unordered_map>
class cLogger
{	
public:
    float loglevel;

public:
    cLogger();
    virtual ~cLogger();
    virtual bool getLog(const char* FileName) = 0;
    virtual void saveLog() = 0;
    virtual void writeToLogMap(const cMap &Map,const cList &path) = 0;
    virtual void writeToLogOpenClose(const cList *open, const std::unordered_multimap<int, Node>& close, const int size) = 0;
    virtual void writeToLogPath(const cList &path, const std::vector<float> &angles) = 0;
    virtual void writeToLogHpLevel(const cList &path) = 0;
    virtual void writeToLogSummary(const cList &path, int numberofsteps, int nodescreated, float length,long double Time, float maxAngle, int sections) = 0;};

#endif
