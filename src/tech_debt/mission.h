#ifndef MISSION_H
#define MISSION_H

#include "config.h"
#include "liansearch.h"
#include "map.h"
#include "search.h"
#include "search_result.h"


#include <string>

class Mission {

public:
    Mission(const char* fName);
    ~Mission();

    bool getConfig();
    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();

private:
    Config      config;
    Map         map;

    Search      *search;
    Logger      *logger;

    const char* fileName;

    SearchResult sr;
};

#endif

