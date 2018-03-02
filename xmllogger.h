#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include "gl_const.h"
#include "logger.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <iostream>
#include <sstream>
#include <string>

class XmlLogger : public Logger {

private:
    std::string     logFileName;
    TiXmlDocument   *doc;

public:
    XmlLogger(float loglvl);
    ~XmlLogger();

    bool getLog(const char* FileName);
    void saveLog();
    void writeToLogIter(int closeSize, const Node &curNode);
    void writeToLogMap(const Map &map,const std::list<Node> &path);
    void writeToLogOpenClose(const OpenList &open, const std::unordered_multimap<int, Node> &close);
    void writeToLogPath(const std::list<Node> &path, const std::vector<float> &angles);
    void writeToLogHpLevel(const std::list<Node> &path);
    void writeToLogSummary(const std::list<Node> &path, int numberofsteps, int nodescreated, float length, float length_scaled,
                           long double time, float max_angle, float accum_angle, int sections);
};

#endif
