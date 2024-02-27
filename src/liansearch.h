#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "structures/map.h"
#include "structures/node.h"
#include "structures/search_tree.h"
#include "structures/search_result.h"
#include "structures/config.h"
#include "log/logger.h"

#include <chrono>
#include <cmath>
#include <list>
#include <limits>
#include <vector>
#include <memory>

class LianSearch {
public:
    LianSearch(SearchParams settings);

    SearchResult startSearch(std::shared_ptr<Logger> logger, const Map &map);

private:
    SearchParams settings_;
    SearchResult sresult_;
    const std::vector<int> distanceLookup_; // Possible step sizes, from largest to smallest
    std::vector<std::vector<circleNode> > circleNodes_; // Virtual nodes that create circle around the cell
    std::vector<std::pair<int,int> > pivotCircle_;  // Vector of nodes (shifts) for pivot security check
    std::vector<float> angles_;
    std::list<Node> lppath_, hppath_; // Final path in two representations
    SearchTree search_tree_; // consists of `open` (list of nodes waiting for expanding) \
                             // and `close` (list of nodes that were already expanded)

     // Called on construction
    std::vector<int> buildDistances() const;

    // Method that calculate Bresenham's Circle (center - (0, 0)) and writing list of created nodes to circleNodes
    void calculateCircle(int radius); // Radius - radius of the circle in cells

    void calculatePivotCircle();
    static std::vector<Node> calculateLineSegment(const Node &start, const Node &goal); // Method builds Bresenham's Line
    static bool checkLineSegment(const Map &map, const Node &start, const Node &goal); // Method builds Bresenham's Line and check it for impassable parts

    // check that there are no obstacle in a safety radius from a turn point
    bool checkPivotCircle(const Map &map, const Node &center);

    static double calcAngle(const Node &dad, const Node &node, const Node &son);
    bool checkAngle(const Node &dad, const Node &node, const Node &son) const;

    bool stopCriterion(); // Check for the ending criteria. Return true if the algorithm should be stopped

    bool tryIncreaseRadius(Node &node);
    bool tryDecreaseRadius(Node &node);

    void update(const Node& current_node, Node &new_node, bool &successors, const Map &map);
    bool expand(const Node& curNode, const Map &map);
    std::list<Node> smoothPath(const std::list<Node>& path, const Map& map);
    void makePrimaryPath(Node* curNode);
    void makeSecondaryPath();
    double makeAngles();
};

#endif // LIANSEARCH_H
