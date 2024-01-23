#include"structures/mission.h"

#include <iostream>

int main(int argc, char* argv[]) {
     if (argc != 2) {
         return 1;
     }
    Mission Mission(argv[1]);

    Mission.startSearch();

    Mission.printSearchResultsToConsole();

    Mission.saveSearchResultsToLog();
    return 0;
}
