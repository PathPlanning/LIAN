#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "gl_const.h"
#include "map.h"
#include "node.h"
#include "openlist.h"
#include "search.h"

#include <chrono>
#include <cmath>
#include <list>
#include <limits>
#include <unordered_map>
#include <vector>

class LianSearch : public Search {

public:

    // Constructor with parameters
    LianSearch(float angleLimit_, int distance_, float weight_, int breakingties_,
               unsigned int steplimit_, float curvatureHeuristicWeight_, bool postsmoother_,
               float decreaseDistanceFactor_, int distanceMin_, double PivotRadius_, int numOfParentsToIncreaseRadius_);

    ~LianSearch();
    SearchResult startSearch(Logger *Log, const Map &map); // General searching algorithm

private:

    float angleLimit; // Maximal value of deviation angle (turning limit)

    int distance; // Minimal value of length of steps

    int numOfParentsToIncreaseRadius;

    std::vector<int> listOfDistances;
    int listOfDistancesSize;

    float weight;  // Heuristics weight

    int BT;

    bool postsmoother; // Smoothing the path after the algorithm

    // Heurisic coefficient:
    // During check for position of goal cell in the circle with minimal radius,
    // squared radius is multiplied by this coefficient
    float circleRadiusFactor;

    // Heurisic coefficient:
    // If there is heuristic that checks deviation of trajectory from line on each
    // step, this deviation is multiplyed by this coefficient
    float curvatureHeuristicWeight;

    float pivotRadius; // Radius of safety circle around every turn point.

    unsigned int stepLimit; // Maximum number of iterations, allowed for the algorithm

    unsigned int closeSize; // Number of elements in close (elements that were already examined)

    float decreaseDistanceFactor; // Value for decreasing the initial distance value
    int distanceMin; // Minimal distance value

    std::vector< std::vector<Node> > circleNodes; // Virtual nodes that create circle around the cell

    std::vector<Node> pivotCircle;  // Vector of nodes (shifts) for pivot security check

    std::vector<float> angles;

    std::list<Node> lppath, hppath; // Final path in two representations
    OpenList open; // Open : list of nodes waiting for expanding

    std::unordered_multimap<int, Node> close; // Close: list of nodes that were already expanded

    // Method that calculate Bresenham's Circle (center - (0, 0)) and writing list of created nodes to circleNodes
    void calculateCircle(int radius); // Radius - radius of the circle in cells

    void calculatePivotCircle();

    int calculatePreferableRadius(const Map &map); // Method calculates the most preferable radius depending on the parameters of the initial map

    void calculateDistances();

    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal); // Method builds Bresenham's Line

    bool checkLineSegment(const Map &map, const Node &start, const Node &goal); // Method builds Bresenham's Line and check it for impassable parts

    // check that there are no obstacle in a safety radius from a turn point
    bool checkPivotCircle(const Map &map, const Node &center);

    double getCost(int a_i, int a_j, int b_i, int b_j) const;

    double calcAngle(const Node &dad, const Node &node, const Node &son) const;
    bool checkAngle(const Node &dad, const Node &node, const Node &son) const;

    bool stopCriterion(); // Check for the ending criteria. Return true if the algorithm should be stopped

    int tryToIncreaseRadius(Node curNode);
    bool tryToDecreaseRadius(Node &curNode, int width);
    void findSuccessors(const Node curNode,std::vector<Node> &successors, const Map &map);
    bool expand(const Node curNode, const Map &map);
    std::list<Node> smoothPath(const std::list<Node>& path, const Map& map);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    double makeAngles();
};

#endif // LIANSEARCH_H
