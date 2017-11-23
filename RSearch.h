#ifndef RSEARCH_H
#define RSEARCH_H

#include "sNode.h"
#include "cList.h"
#include "cMap.h"
#include "cSearch.h"
#include <vector>

#define STEP_LIMIT 1000000;

class RSearch : public cSearch
{
public:
    RSearch(float weight, int MT, int BT, int DELTA, int ASTARMAXEXPANDS, int NUMOFNODE, int CLD=CN_LM_CLOSED);
    ~RSearch();

    SearchResult startSearch(cLogger *Log, cMap &Map);

private:
    cList openA, closeA, pathA;
    cList open, close, path;

    std::vector<Node> circle;

    cList succs;

    char *message;

    float weight;
    int metrictype;
    int closedlistmode;
    int breakingties;
    unsigned int m_maxNodes;

    int currentStepNumber;
    bool currentStepMode;

    cMap m_aMap;
    cMap m_Map;

    int m_delta;
    int m_astarMaxExpands;
    int m_numOfNode;

    int     calculateDistance(int currI, int newI);
    void    addOpen(Node& newNode);
    float   computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j);
    void    reevaluateState(Node newNode);
    int     calculateTeDistanceForJ(Node curNode, int succi);
    int     calculateTeDistanceForI(Node curNode, int succj);
    void    addOpenAstar(Node& newNode);
    int    startASearch(bool nodeIsNormal);
    void    calculateCircle();
    bool   emergencyStopCriterion();

};

#endif
