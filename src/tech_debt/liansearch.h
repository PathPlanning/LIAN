#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "gl_const.h"
#include "map.h"
#include "node.h"
#include "open_list.h"
#include "search_result.h"
#include "config.h"
#include "logger.h"

#include <chrono>
#include <cmath>
#include <list>
#include <limits>
#include <unordered_map>
#include <vector>
#include <memory>

class LianSearch {
public:
    LianSearch(SearchParams settings);

    SearchResult startSearch(std::shared_ptr<Logger> logger, const Map &map);

private:
    SearchParams settings_;

    SearchResult sresult_;

    const std::vector<int> listOfDistances_;

    std::vector<int> buildDistances();

    std::vector<std::vector<circleNode> > circleNodes_; // Virtual nodes that create circle around the cell

    std::vector<std::pair<int,int> > pivotCircle_;  // Vector of nodes (shifts) for pivot security check

    std::vector<float> angles_;

    std::list<Node> lppath_, hppath_; // Final path in two representations
    OpenList open_; // Open : list of nodes waiting for expanding

    std::unordered_multimap<int, Node> close_; // Close: list of nodes that were already expanded

    // Method that calculate Bresenham's Circle (center - (0, 0)) and writing list of created nodes to circleNodes
    void calculateCircle(int radius); // Radius - radius of the circle in cells

    void calculatePivotCircle();

    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal); // Method builds Bresenham's Line

    bool checkLineSegment(const Map &map, const Node &start, const Node &goal); // Method builds Bresenham's Line and check it for impassable parts

    // check that there are no obstacle in a safety radius from a turn point
    bool checkPivotCircle(const Map &map, const Node &center);

    static double calcAngle(const Node &dad, const Node &node, const Node &son);
    bool checkAngle(const Node &dad, const Node &node, const Node &son) const;

    bool stopCriterion(); // Check for the ending criteria. Return true if the algorithm should be stopped

    int tryToIncreaseRadius(Node* curNode);
    bool tryToDecreaseRadius(Node &curNode, int width);
    void update(const Node current_node, Node new_node, bool &successors, const Map &map);
    bool expand(const Node curNode, const Map &map);
    std::list<Node> smoothPath(const std::list<Node>& path, const Map& map);
    void makePrimaryPath(Node* curNode);
    void makeSecondaryPath();
    double makeAngles();
};

#endif // LIANSEARCH_H
