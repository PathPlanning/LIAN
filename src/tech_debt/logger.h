#pragma once

#include <string>

#include "gl_const.h"
#include "tinyxml/tinyxml.h"

class Logger {
public:
    Logger(int loglvl, const std::string& configFileName);
    Logger(const Logger& oth) = delete;
    Logger(Logger&& oth) = delete;
    Logger& operator=(const Logger& oth) = delete;
    Logger& operator=(Logger&& oth) = delete;
    ~Logger();

    template<int loglvl>
    TiXmlElement* logSpace(const std::string& tagName) {
        if (logLevel_ < loglvl) {
            return nullptr;
        }
        return doc_.FirstChild(CNS_TAG_LOG)->FirstChildElement(tagName.c_str());
    }
private:
    int logLevel_;
    std::string logFileName_;
    TiXmlDocument doc_;
};
