#ifndef MISSION_H
#define MISSION_H

#include "config.h"
#include "../liansearch.h"
#include "../log/logger.h"
#include "map.h"
#include "search_result.h"

#include <string>
#include <memory>

class Mission {

public:
    Mission(const char* fName);

    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();

private:
    Config config;
    Map map;

    LianSearch search;
    std::shared_ptr<Logger> logger;

    const char* fileName;

    SearchResult sr;
};

#endif

