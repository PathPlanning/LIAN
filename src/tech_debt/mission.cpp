#include "mission.h"

namespace {
    void saveSummaryToLog(Logger* logger, const std::list<Node>& path, int numberofsteps, int nodescreated, float length, float length_scaled,
        long double time, float max_angle, float accum_angle, int sections) {
        auto space = logger->logSpace<CN_LOGLVL_TINY>(CNS_TAG_SUM);
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
            space->SetAttribute(CNS_TAG_ATTR_PF, CNS_TAG_ATTR_FALSE);
        }
        else {
            space->SetAttribute(CNS_TAG_ATTR_PF, CNS_TAG_ATTR_TRUE);
        }

        space->SetAttribute(CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps);
        space->SetAttribute(CNS_TAG_ATTR_NODESCREATED, nodescreated);
        space->SetAttribute(CNS_TAG_ATTR_SECTIONS, sections);
        space->SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, length);
        space->SetDoubleAttribute(CNS_TAG_ATTR_LENGTHSC, length_scaled);
        space->SetAttribute(CNS_TAG_ATTR_TIME, timeValue.c_str());
        space->SetDoubleAttribute(CNS_TAG_ATTR_MAXANGLE, max_angle);
        space->SetDoubleAttribute(CNS_TAG_ATTR_ACCUMANGLE, accum_angle);
    }

    void savePathToLog(Logger* logger, const std::list<Node>& path, const std::vector<float>& angles) {
        auto space = logger->logSpace<CN_LOGLVL_HIGH>(CNS_TAG_LPLEVEL);
        if (!space) {
            return;
        }

        TiXmlElement* point;
        int64_t index = 0;
        for (auto iter = path.cbegin(); iter != path.cend(); ++iter, ++index) {
            TiXmlElement point(CNS_TAG_NODE);
            point.SetAttribute(CNS_TAG_ATTR_NUM, index);
            point.SetAttribute(CNS_TAG_ATTR_X, iter->j);
            point.SetAttribute(CNS_TAG_ATTR_Y, iter->i);
            space->InsertEndChild(point);
        }

        if (angles.size() == 0) return;

        space = logger->logSpace<CN_LOGLVL_HIGH>(CNS_TAG_ANGLES);

        std::vector<float>::const_iterator iter2 = angles.end();
        for (auto iter = angles.crbegin(); iter != angles.crend(); ++iter) {
            TiXmlElement point(CNS_TAG_ANGLE);
            point.SetAttribute(CNS_TAG_ATTR_NUM, (iter - angles.crbegin()));
            point.SetDoubleAttribute(CNS_TAG_ATTR_VALUE, *iter);
            space->InsertEndChild(point);
        }
    }

    void saveMapToLog(Logger* logger, const Map& map, const std::list<Node>& path) {
        auto space = logger->logSpace<CN_LOGLVL_HIGH>(CNS_TAG_PATH);
        if (!space) {
            return;
        }

        std::stringstream stream;
        std::string text, value;
        std::vector<int> curLine(map.getWidth(), 0);

        for (int i = 0; i < map.getHeight(); i++) {
            TiXmlElement msg(CNS_TAG_ROW);
            msg.SetAttribute(CNS_TAG_ATTR_NUM, i);
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

    void saveToLogHpLevel(Logger* logger, const std::list<Node>& path) {
        auto space = logger->logSpace<CN_LOGLVL_HIGH>(CNS_TAG_HPLEVEL);
        if (!space) {
            return;
        }
        int partnumber = 0;
        TiXmlElement* part;
        auto it = path.cbegin();

        for (auto iter = ++path.cbegin(); iter != path.cend(); ++iter, ++it) {
            TiXmlElement part(CNS_TAG_SECTION);
            part.SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
            part.SetAttribute(CNS_TAG_ATTR_SX, it->j);
            part.SetAttribute(CNS_TAG_ATTR_SY, it->i);
            part.SetAttribute(CNS_TAG_ATTR_FX, iter->j);
            part.SetAttribute(CNS_TAG_ATTR_FY, iter->i);
            part.SetDoubleAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
            space->InsertEndChild(part);
            ++partnumber;
        }
    }
}

Mission::Mission(const char* fName) : fileName(fName), config(fName), map(fName), search(nullptr), logger(nullptr) {}

Mission::~Mission() {
    delete search;
    delete logger;
}

void Mission::createSearch() {
    search = new LianSearch(config.params().angleLimit,
                            config.params().distance,
                            config.params().weight,
                            config.params().steplimit,
                            config.params().curvatureHeuristicWeight,
                            config.params().postsmoother,
                            config.params().decreaseDistanceFactor,
                            config.params().distanceMin,
                            config.params().pivotRadius,
                            config.params().numOfParentsToIncreaseRadius);
}

bool Mission::createLog() {
    if(config.params().logLevel == CN_LOGLVL_LOW || config.params().logLevel == CN_LOGLVL_HIGH ||
        config.params().logLevel == CN_LOGLVL_MED || config.params().logLevel == CN_LOGLVL_TINY ||
        config.params().logLevel - CN_LOGLVL_ITER < 0.001) {
        logger = new Logger(config.params().logLevel);
    }
    else if (config.params().logLevel == CN_LOGLVL_NO) {
        logger = new Logger(config.params().logLevel);
        return true;
    } else {
        std::cout << "'loglevel' is not correctly specified in input XML-file.\n";
        return false;
    }
    return logger->getLog(fileName);
}

void Mission::startSearch() {
    sr = search->startSearch(logger, map);
}

void Mission::printSearchResultsToConsole() {
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "length_scaled=" << sr.pathlength * map.getCellSize() << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog() {
    saveSummaryToLog(logger, sr.hppath, sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.pathlength * map.getCellSize(),
                              sr.time, sr.max_angle, sr.accum_angle, sr.sections);

    if (sr.pathfound) {
        savePathToLog(logger, sr.lppath, sr.angles);
        saveMapToLog(logger, map, sr.lppath);
        saveToLogHpLevel(logger, sr.hppath);
    }
    logger->saveLog();
}

