#include"mission.h"
#include"rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc == 2) {
        Mission Mission(argv[1]);
        std::cout<<"Retreiving map from input XML file.\n";
        if (!Mission.getMap()) {
            std::cout<<"Program terminated.\n";
            return 0;
        }

        std::cout<<"Retreiving search algorithm configuration from input XML file.\n";
        if (!Mission.getConfig()) return 0;

        Mission.createSearch();
        Mission.createLog();
        Mission.startSearch();

        std::cout<<"Search is finished!"<<std::endl;

        Mission.printSearchResultsToConsole();

        Mission.saveSearchResultsToLog();
        std::cout<<"Results are saved (if chosen) via created log channel."<<std::endl;
    }
    return 1;
}
