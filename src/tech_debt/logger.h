#pragma once

#include <string>

#include "gl_const.h"
#include "tinyxml/tinyxml.h"

class Logger {
public:
    Logger(float loglvl);
    ~Logger();

    bool getLog(const std::string& FileName);
    void saveLog();

    template<float loglvl>
    TiXmlElement* logSpace(const std::string& tagName) {
        if (logLevel_ < loglvl) {
            return nullptr;
        }
        return doc_->FirstChild(CNS_TAG_ROOT)->FirstChild(CNS_TAG_LOG)->FirstChildElement(tagName.c_str());
    }

    template<>
    TiXmlElement* logSpace<CN_LOGLVL_ITER>(const std::string& tagName) {
        if (logLevel_ != CN_LOGLVL_ITER) {
            return nullptr;
        }
        return doc_->FirstChild(CNS_TAG_ROOT)->FirstChild(CNS_TAG_LOG)->FirstChildElement(tagName.c_str());
    }

private:
    float logLevel_;
    std::string logFileName_;
    TiXmlDocument* doc_ = nullptr;
};
