#include "mission.h"

namespace {
    void saveSummaryToLog(std::shared_ptr<Logger> logger, const std::list<Node>& path, int numberofsteps, int nodescreated, float length, float length_scaled,
        long double time, float max_angle, float accum_angle, int sections) {
        auto space = logger->logSpace<Logger::Levels::tiny>(Logger::Tags::summary);
        if (!space) {
            return;
        }

        std::string timeValue;
        std::stringstream stream;
        stream << time;
        stream >> timeValue;
        stream.clear();
        stream.str("");

        if (path.size() == 0) {
            space->SetAttribute(Logger::Tags::pathFound, Logger::Tags::tagFalse);
        }
        else {
            space->SetAttribute(Logger::Tags::pathFound, Logger::Tags::tagTrue);
        }

        space->SetAttribute(Logger::Tags::numberOfSteps, numberofsteps);
        space->SetAttribute(Logger::Tags::nodesCreated, nodescreated);
        space->SetAttribute(Logger::Tags::sections, sections);
        space->SetDoubleAttribute(Logger::Tags::length, length);
        space->SetDoubleAttribute(Logger::Tags::lengthScaled, length_scaled);
        space->SetAttribute(Logger::Tags::time, timeValue.c_str());
        space->SetDoubleAttribute(Logger::Tags::maxAngle, max_angle);
        space->SetDoubleAttribute(Logger::Tags::accumAngle, accum_angle);
    }

    void savePathToLog(std::shared_ptr<Logger> logger, const std::list<Node>& path, const std::vector<float>& angles) {
        auto space = logger->logSpace<Logger::Levels::high>(Logger::Tags::lpLevel);
        if (!space) {
            return;
        }

        int64_t index = 0;
        for (auto iter = path.cbegin(); iter != path.cend(); ++iter, ++index) {
            TiXmlElement point(Logger::Tags::node);
            point.SetAttribute(Logger::Tags::number, index);
            point.SetAttribute(Logger::Tags::parentX, iter->j);
            point.SetAttribute(Logger::Tags::y, iter->i);
            space->InsertEndChild(point);
        }

        if (angles.size() == 0) return;

        space = logger->logSpace<Logger::Levels::high>(Logger::Tags::angles);

        for (auto iter = angles.crbegin(); iter != angles.crend(); ++iter) {
            TiXmlElement point(Logger::Tags::angle);
            point.SetAttribute(Logger::Tags::number, (iter - angles.crbegin()));
            point.SetDoubleAttribute(Logger::Tags::value, *iter);
            space->InsertEndChild(point);
        }
    }

    void saveMapToLog(std::shared_ptr<Logger> logger, const Map& map, const std::list<Node>& path) {
        auto space = logger->logSpace<Logger::Levels::high>(Logger::Tags::path);
        if (!space) {
            return;
        }

        std::stringstream stream;
        std::string text, value;
        std::vector<int> curLine(map.getWidth(), 0);

        for (int i = 0; i < map.getHeight(); i++) {
            TiXmlElement msg(Logger::Tags::row);
            msg.SetAttribute(Logger::Tags::number, i);
            text = "";

            for (auto iter = path.begin(); iter != path.end(); ++iter) {
                if (iter->i == i) {
                    curLine[iter->j] = 1;
                }
            }

            for (int j = 0; j < map.getWidth(); j++) {
                if (curLine[j] != 1) {
                    stream << map[i][j];
                    stream >> value;
                    stream.clear();
                    stream.str("");
                    text = text + value + " ";
                }
                else {
                    text = text + "*" + " ";
                    curLine[j] = 0;
                }
            }

            msg.InsertEndChild(TiXmlText(text.c_str()));
            space->InsertEndChild(msg);
        }
    }

    void saveToLogHpLevel(std::shared_ptr<Logger> logger, const std::list<Node>& path) {
        auto space = logger->logSpace<Logger::Levels::high>(Logger::Tags::hpLevel);
        if (!space) {
            return;
        }
        int partnumber = 0;
        auto it = path.cbegin();

        for (auto iter = ++path.cbegin(); iter != path.cend(); ++iter, ++it) {
            TiXmlElement part(Logger::Tags::section);
            part.SetAttribute(Logger::Tags::number, partnumber);
            part.SetAttribute(Logger::Tags::startX, it->j);
            part.SetAttribute(Logger::Tags::startY, it->i);
            part.SetAttribute(Logger::Tags::finishX, iter->j);
            part.SetAttribute(Logger::Tags::finishY, iter->i);
            part.SetDoubleAttribute(Logger::Tags::length, iter->g - it->g);
            space->InsertEndChild(part);
            ++partnumber;
        }
    }
}

Mission::Mission(const char* fName) 
    : config(fName)
    , map(fName)
    , search(config.params())
    , logger(std::make_shared<Logger>(config.params().logLevel, std::string(fName)))
    , fileName(fName) {}

void Mission::startSearch() {
    sr = search.startSearch(logger, map);
}

void Mission::printSearchResultsToConsole() {
    std::cout << "Path ";
    if (!sr.pathFound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "nodescreated=" << sr.nodesCreated << std::endl;
    std::cout << "numberofsteps=" << sr.numberOfSteps << std::endl;
    if (sr.pathFound) {
        std::cout << "pathlength=" << sr.pathLength << std::endl;
        std::cout << "length_scaled=" << sr.pathLength * map.getCellSize() << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog() {
    saveSummaryToLog(logger, sr.hpPath, sr.numberOfSteps, sr.nodesCreated, sr.pathLength, sr.pathLength * map.getCellSize(),
                     sr.time, sr.maxAngle, sr.accumAngle, sr.sections);

    if (sr.pathFound) {
        savePathToLog(logger, sr.lpPath, sr.angles);
        saveMapToLog(logger, map, sr.lpPath);
        saveToLogHpLevel(logger, sr.hpPath);
    }
}

