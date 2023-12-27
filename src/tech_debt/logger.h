#pragma once

#include <string>

#include "gl_const.h"
#include "tinyxml/tinyxml.h"

class Logger {
public:
    Logger(int loglvl);
    ~Logger();

    bool getLog(const std::string& FileName);
    void saveLog();

    template<int loglvl>
    TiXmlElement* logSpace(const std::string& tagName) {
        if (logLevel_ < loglvl) {
            return nullptr;
        }
        return doc_->FirstChild(CNS_TAG_ROOT)->FirstChild(CNS_TAG_LOG)->FirstChildElement(tagName.c_str());
    }
private:
    int logLevel_;
    std::string logFileName_;
    TiXmlDocument* doc_ = nullptr;
};
