#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include "cList.h"
#include <vector>

struct SearchResult
{
        bool pathfound;
        float pathlength;
        cList hppath,lppath;
        unsigned int nodescreated;
        unsigned int numberofsteps;
        std::vector<float> angles;
        double time;
        float maxAngle;
        int sections;

        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
            hppath.List.clear();
            lppath.List.clear();
            angles.clear();
            nodescreated = 0;
            numberofsteps = 0;
            time = 0;
            maxAngle = 0;
            sections=0;
        }

};

#endif // SEARCHRESULT_H
