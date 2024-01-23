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
        return doc_.FirstChild(Logger::Tags::log)->FirstChildElement(tagName.c_str());
    }

    struct Tags {
        static constexpr const char
            *root = "root",
            *row = "row",
            *lpLevel = "lpLevel",
            *hpLevel = "hpLevel",
            *log = "log",
            *lowLevel = "lowLevel",
            *mapFileName = "mapFileName",
            *summary = "summary",
            *iterations = "iterations",
            *section = "section",
            *step = "step",
            *open = "open",
            *close = "close",
            *path = "path",
            *time = "time",
            *y = "Y",
            *F = "F",
            *g = "g",
            *angle = "angle",
            *angles = "angles",
            *finishX = "finish.x",
            *finishY = "finish.y",
            *length = "length",
            *lengthScaled = "length_scaled",
            *node = "node",
            *nodesCreated = "nodescreated",
            *number = "number",
            *numberOfSteps = "numberofsteps",
            *pathFound = "pathfound",
            *parentX = "parent_x",
            *parentY = "parent_y",
            *sections = "sections",
            *startX = "start.x",
            *startY = "start.y",
            *value = "value",
            *tagTrue = "true",
            *tagFalse = "false",
            *maxAngle = "max_angle",
            *accumAngle = "accum_angle";
    };

    struct Levels {
        static constexpr int
            no = 0,
            tiny = 1,
            high = 2,
            iter = 3,
            med = 4,
            low = 5;
    };

private:
    int logLevel_;
    std::string logFileName_;
    TiXmlDocument doc_;
};
