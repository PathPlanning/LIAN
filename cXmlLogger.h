#ifndef CXMLLOGGER_H
#define CXMLLOGGER_H

#include"cLogger.h"
#include<iostream>
#include"tinyxml.h"
#include"tinystr.h"
#include<string>


class cXmlLogger:public cLogger
{
private:
    std::string LogFileName;
    TiXmlDocument *doc;

public:
    cXmlLogger(float loglvl);
    ~cXmlLogger();

    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogMap(const cMap &Map,const cList &path);
    void writeToLogOpenClose(const cList *open, const std::unordered_multimap<int, Node> &close, const int size);
    void writeToLogPath(const cList &path, const std::vector<float> &angles);
    void writeToLogHpLevel(const cList &path);
    void writeToLogSummary(const cList &path, int numberofsteps, int nodescreated, float length, long double Time, float maxAngle, int sections);
};

#endif
