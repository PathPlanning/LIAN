#include "mission.h"

Mission::Mission(const char* fName) : fileName(fName), search(nullptr), logger(nullptr) {}

Mission::~Mission() {
    delete search;
    delete logger;
}

bool Mission::getMap() {
    return map.getMap(fileName);
}

bool Mission::getConfig() {
    return config.getConfig(fileName);
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
        logger = new XmlLogger(config.params().logLevel);
    }
    else if (config.params().logLevel == CN_LOGLVL_NO) {
        logger = new XmlLogger(config.params().logLevel);
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
    logger->writeToLogSummary(sr.hppath, sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.pathlength * map.getCellSize(),
                              sr.time, sr.max_angle, sr.accum_angle, sr.sections);

    if (sr.pathfound) {
        logger->writeToLogPath(sr.lppath, sr.angles);
        logger->writeToLogMap(map,sr.lppath);
        logger->writeToLogHpLevel(sr.hppath);
    }
    logger->saveLog();
}

