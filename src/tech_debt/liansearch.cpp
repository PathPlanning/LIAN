#include "liansearch.h"

namespace {
    void saveIterationToLog(Logger* logger, int closeSize_, const Node& curNode) {
        auto space = logger->logSpace<CN_LOGLVL_ITER>(CNS_TAG_ITERS);
        if (!space) {
            return;
        }

        TiXmlElement element(CNS_TAG_STEP);

        element.SetAttribute(CNS_TAG_STEP, closeSize_);
        element.SetAttribute(CNS_TAG_ATTR_X, curNode.j);
        element.SetAttribute(CNS_TAG_ATTR_Y, curNode.i);
        if (curNode.parent) {
            element.SetAttribute(CNS_TAG_ATTR_PARX, curNode.parent->j);
            element.SetAttribute(CNS_TAG_ATTR_PARY, curNode.parent->i);
        }
        element.SetDoubleAttribute(CNS_TAG_ATTR_F, curNode.F);
        element.SetDoubleAttribute(CNS_TAG_ATTR_G, curNode.g);

        space->InsertEndChild(element);
    }

    void saveToLogOpenAndClose(Logger* logger, const OpenList& open_,
        const std::unordered_multimap<int, Node>& close_) {
        auto space = logger->logSpace<CN_LOGLVL_LOW>(CNS_TAG_LOWLEVEL);
        if (!space) {
            return;
        }

        int iterate = 0;
        TiXmlNode* child = nullptr, * curNode = space;

        while ((child = curNode->IterateChildren(child)))
            iterate++;

        {
            TiXmlElement element(CNS_TAG_STEP);
            element.SetAttribute(CNS_TAG_ATTR_NUM, iterate);
            curNode->InsertEndChild(element);
            curNode = curNode->LastChild();
        }

        {

            TiXmlElement element(CNS_TAG_OPEN);
            curNode->InsertEndChild(element);
            child = curNode->LastChild();
        }

        open_.writeToXml(child);

        {
            TiXmlElement element(CNS_TAG_CLOSE);
            curNode->InsertEndChild(element);
            child = curNode->LastChild();
        }

        for (const auto& it : close_) {
            TiXmlElement element(CNS_TAG_NODE);
            element.SetAttribute(CNS_TAG_ATTR_X, it.second.j);
            element.SetAttribute(CNS_TAG_ATTR_Y, it.second.i);
            element.SetDoubleAttribute(CNS_TAG_ATTR_F, it.second.F);
            element.SetDoubleAttribute(CNS_TAG_ATTR_G, it.second.g);
            if (it.second.g > 0) {
                element.SetAttribute(CNS_TAG_ATTR_PARX, it.second.parent->j);
                element.SetAttribute(CNS_TAG_ATTR_PARY, it.second.parent->i);
            }
            child->InsertEndChild(element);
        }
    }
}

/*
 * // Use for more accurate time calculation
 * #ifdef __linux__
 *     #include <sys/time.h>
 * #else
 *     #include <windows.h>
 * #endif
 *
*/

LianSearch::~LianSearch() = default;


LianSearch::LianSearch(float angleLimit_, int distance_, float weight_,
    unsigned int stepLimit_, float curvatureHeuristicWeight_, bool postSmoother_,
    float decreaseDistanceFactor_, int distanceMin_,
    float pivotRadius_, int numOfParentsToIncreaseRadius_) :
    angleLimit(angleLimit_),
    distance(distance_),
    numOfParentsToIncreaseRadius(numOfParentsToIncreaseRadius_),
    weight(weight_),
    postsmoother(postSmoother_),
    curvatureHeuristicWeight(curvatureHeuristicWeight_),
    pivotRadius(pivotRadius_),
    stepLimit(stepLimit_),
    decreaseDistanceFactor(decreaseDistanceFactor_),
    distanceMin(distanceMin_)
{
    srand(time(nullptr));
}

void LianSearch::calculateCircle(int radius) { //here radius - radius of the circle in cells
    circleNodes.clear();
    circleNodes.resize(listOfDistances.size());
    for (int k = 0; k < listOfDistances.size(); ++k) {
        radius = listOfDistances[k];
        circleNodes[k].clear();
        std::vector<circleNode> circle_nodes(0);
        int x = 0;
        int y = radius;
        int delta = 2 - 2 * radius;
        while (y >= 0) {
            x = std::min(x, radius);
            x = std::max(x, -radius);
            y = std::min(y, radius);
            y = std::max(y, -radius);
            double dist = getCost(0, 0, x, y);
            circle_nodes.emplace_back(x, y, dist);
            circle_nodes.emplace_back(x, -y, dist);
            circle_nodes.emplace_back(-x, y, dist);
            circle_nodes.emplace_back(-x, -y, dist);

            int error = 2 * (delta + y) - 1;
            if (delta < 0 && error <= 0) {
                delta += 2 * ++x + 1;
                continue;
            }

            error = 2 * (delta - x) - 1;
            if (delta > 0 && error > 0) {
                delta += 1 - 2 * --y;
                continue;
            }
            delta += 2 * (++x - y--);
        }

        for (int i = 0; i < circle_nodes.size(); i += 4) {
            circleNodes[k].push_back(circle_nodes[i]);
        }
        for (int i = circle_nodes.size() - 7; i > 0; i -= 4) {
            circleNodes[k].push_back(circle_nodes[i]);
        }
        for (int i = 7; i < circle_nodes.size(); i += 4) {
            circleNodes[k].push_back(circle_nodes[i]);
        }
        for (int i = circle_nodes.size() - 6; i > 0; i -= 4) {
            circleNodes[k].push_back(circle_nodes[i]);
        }
        circleNodes[k].pop_back();
        for (size_t i = 0; i < circleNodes[k].size(); ++i) {
            double angle = acos((circleNodes[k][0].i * circleNodes[k][i].i + circleNodes[k][0].j * circleNodes[k][i].j)
                / (sqrt(pow(circleNodes[k][0].i, 2) + pow(circleNodes[k][0].j, 2))
                    * sqrt(pow(circleNodes[k][i].i, 2) + pow(circleNodes[k][i].j, 2))));
            if (i < circleNodes[k].size() / 2) {
                circleNodes[k][i].heading = angle * 180 / CN_PI_CONSTANT;
            }
            else {
                circleNodes[k][i].heading = 360 - angle * 180 / CN_PI_CONSTANT;
            }
            //std::cout << circleNodes[k][i].heading  << std::endl;
        }
    }
}

void LianSearch::calculatePivotCircle() {
    pivotCircle.clear();
    int add_i;
    int add_j;
    int num = floor(pivotRadius + 0.5 - CN_EPSILON);
    for (int i = -num; i <= +num; i++) {
        for (int j = -num; j <= +num; j++) {
            add_i = i != 0 ? 1 : 0;
            add_j = j != 0 ? 1 : 0;
            if ((pow(2 * abs(i) - add_i, 2) + pow(2 * abs(j) - add_j, 2)) < pow(2 * pivotRadius, 2)) {
                pivotCircle.push_back({ i, j });
            }
        }
    }
    if (pivotCircle.empty()) {
        pivotCircle.push_back({ 0, 0 });
    }
}

bool LianSearch::checkPivotCircle(const Map& map, const Node& center) {
    int i, j;
    for (int k = 0; k < pivotCircle.size(); k++) {
        i = center.i + pivotCircle[k].first;
        j = center.j + pivotCircle[k].second;
        if (!map.CellOnGrid(i, j) || map.CellIsObstacle(i, j)) {
            return false;
        }
    }
    return true;
}

void LianSearch::calculateDistances() {
    int curDistance = distance;
    if (decreaseDistanceFactor > 1) {
        while (curDistance >= distanceMin) {
            listOfDistances.push_back(curDistance);
            curDistance = ceil(curDistance / decreaseDistanceFactor);
        }
    }
    else {
        listOfDistances.push_back(curDistance);
    }
}


template <class ActionF>
bool lineSegmentTraverse(const Node& start, const Node& goal, ActionF action) {
    int64_t x1 = start.i;
    int64_t y1 = start.j;
    int64_t x2 = goal.i;
    int64_t y2 = goal.j;

    int64_t dx = abs(x2 - x1), dy = abs(y2 - y1);
    int64_t stepVal = 0;
    int rotate = 0;

    if (x1 > x2 && y1 > y2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    else if (x2 - x1 >= 0 && y2 - y1 >= 0) {
        rotate = 2;
    }
    else if (y2 - y1 < 0) {
        std::swap(y1, y2);
        rotate = 1;
    }
    else if (x2 - x1 < 0) {
        std::swap(x1, x2);
        rotate = 3;
    }

    int64_t i = 0, j = 0;

    bool iFromX = dx >= dy;

    int64_t stepInc = iFromX ? dy : dx;
    int64_t stepDec = iFromX ? dx : dy;
    int64_t startI = iFromX ? x1 : y1;
    int64_t finishI = iFromX ? x2 : y2;

    bool rotateFirstBit = rotate & 1;
    j = !rotateFirstBit ? (x1 ^ y1 ^ startI) : (x2 ^ y2 ^ finishI);
    int dj = !rotateFirstBit ? 1 : -1;

    for (i = startI; i <= finishI; ++i) {
        if (iFromX ? !action(i, j) : !action(j, i)) {
            break;
        }

        stepVal += stepInc;
        if (stepVal >= stepDec) {
            j += dj;
            stepVal -= stepDec;
        }
    }

    bool reversed = rotate == 0 || (rotate == 1 && !iFromX) || (rotate == 3 && iFromX);
    return reversed;
}

void LianSearch::calculateLineSegment(std::vector<Node>& line, const Node& start, const Node& goal) {
    line.clear();
    bool reversed = lineSegmentTraverse(start, goal,
        [&line](int64_t t, int64_t c) {
            line.push_back(Node(t, c));
            return true;
        });
    if (reversed) {
        std::reverse(line.begin(), line.end());
    }
}

bool LianSearch::checkLineSegment(const Map& map, const Node& start, const Node& goal) {
    bool hasObstacleOnLineSegment = false;
    lineSegmentTraverse(start, goal,
        [&map, &hasObstacleOnLineSegment](int64_t t, int64_t c) {
            hasObstacleOnLineSegment |= map.CellIsObstacle(t, c);
            return !hasObstacleOnLineSegment;
        });
    return !hasObstacleOnLineSegment;
}


bool LianSearch::stopCriterion() {
    if (open.size() == 0) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }

    if (close.size() > stepLimit && stepLimit > 0) {
        std::cout << "Algorithm esceeded step limit!" << std::endl;
        return true;
    }
    return false;
}

double LianSearch::getCost(int a_i, int a_j, int b_i, int b_j) {
    return sqrt(abs(a_i - b_i) * abs(a_i - b_i) + abs(a_j - b_j) * abs(a_j - b_j));
}


double LianSearch::calcAngle(const Node& dad, const Node& node, const Node& son) {
    double cos_angle = (node.j - dad.j) * (son.j - node.j) +
        (node.i - dad.i) * (son.i - node.i);
    cos_angle /= getCost(son.i, son.j, node.i, node.j);
    cos_angle /= getCost(node.i, node.j, dad.i, dad.j);

    cos_angle = std::min(cos_angle, (double)1);
    cos_angle = std::max(cos_angle, (double)-1);

    return acos(cos_angle);
}

SearchResult LianSearch::startSearch(Logger* Log, const Map& map) {

    calculateDistances();

    std::cout << "List of distances :";
    for (auto dist : listOfDistances) {
        std::cout << " " << dist;
    }
    std::cout << std::endl;

    open.resize(map.getHeight());
    Node curNode(map.start_i, map.start_j, 0.0, 0.0, 0.0);
    curNode.radius = distance;
    curNode.F = weight * getCost(curNode.i, curNode.j, map.goal_i, map.goal_j);
    bool pathFound = false;
    open.add(curNode);
    calculateCircle((int)curNode.radius);
    calculatePivotCircle();

    std::chrono::time_point<std::chrono::system_clock> begin, end;
    begin = std::chrono::system_clock::now();

    /*
     * #ifdef __linux__
     *     timeval begin, end;
     *     gettimeofday(&begin, NULL);
     * #else
     *     LARGE_INTEGER begin,end,freq;
     *     QueryPerformanceCounter(&begin);
     *     QueryPerformanceFrequency(&freq);
     * #endif
     */

    while (!stopCriterion()) { // main cycle of the search
        curNode = open.getMin();
        close.insert({ curNode.convolution(map.getWidth()),curNode });

        saveIterationToLog(Log, close.size(), curNode);

        if (curNode.i == map.goal_i && curNode.j == map.goal_j) { // if current point is goal point - end of the cycle
            pathFound = true;
            break;
        }

        if (!expand(curNode, map) && listOfDistances.size() > 1)
            while (curNode.radius > listOfDistances[listOfDistances.size() - 1])
                if (tryToDecreaseRadius(curNode, map.getWidth()))
                    if (expand(curNode, map))
                        break;

        saveToLogOpenAndClose(Log, open, close);
    }

    saveToLogOpenAndClose(Log, open, close);

    sresult.nodesCreated = open.size() + close.size();
    sresult.numberOfSteps = close.size();
    if (pathFound) {
        sresult.pathLength = curNode.g;
        makePrimaryPath(&curNode);
        if (postsmoother) {
            hppath = smoothPath(hppath, map);
        }
        makeSecondaryPath();
        float max_angle = makeAngles();
        sresult.pathFound = true;
        sresult.hpPath = hppath;
        sresult.lpPath = lppath;
        sresult.angles = angles;
        sresult.maxAngle = max_angle;
        sresult.sections = hppath.size() - 1;

        end = std::chrono::system_clock::now();
        sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;
        /* // for more accurate time calculation
       #ifdef __linux__
           gettimeofday(&end, NULL);
           sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
       #else
           QueryPerformanceCounter(&end);
           sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
       #endif */

        return sresult;
    }
    else {
        sresult.pathFound = false;

        end = std::chrono::system_clock::now();
        sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;

        /* for more accurate time calculation
       #ifdef __linux__
           gettimeofday(&end, NULL);
           sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
       #else
           QueryPerformanceCounter(&end);
           sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
       #endif */

        return sresult;
    }
}

int LianSearch::tryToIncreaseRadius(Node* curNode) {
    bool change = false;
    std::size_t i, k = 0;
    while (k < numOfParentsToIncreaseRadius) {
        if (!(curNode->parent != nullptr && curNode->radius == curNode->parent->radius)) {
            break;
        }
        ++k;
        curNode = curNode->parent;
    }
    if (k == numOfParentsToIncreaseRadius) {
        for (i = listOfDistances.size(); i >= 1; --i) {
            if (curNode->radius == listOfDistances[i - 1]) {
                break;
            }
        }
        change = i > 1;
    }
    if (change) {
        return listOfDistances[i - 2];
    }
    return curNode->radius;
}

void LianSearch::update(const Node current_node, Node new_node, bool& successors, const Map& map) {
    if (!checkLineSegment(map, *new_node.parent, new_node)) {
        return;
    }
    if (pivotRadius > 0 && (new_node.i != map.goal_i || new_node.j != map.goal_j)) {
        if (!checkPivotCircle(map, new_node)) {
            return;
        }
    }

    auto it = close.find(new_node.convolution(map.getWidth()));
    if (it != close.end()) {
        auto range = close.equal_range(it->first);
        for (auto it = range.first; it != range.second; ++it) {
            if (it->second.parent == nullptr) {
                return;
            }
            if (it->second.parent->i == current_node.i && it->second.parent->j == current_node.j) {
                return;
            }
        }
    }

    if (listOfDistances.size() > 1) {
        new_node.radius = tryToIncreaseRadius(&new_node);
    }
    open.add(new_node);
    successors = true;
}

bool LianSearch::expand(const Node curNode, const Map& map) {
    std::size_t current_distance;
    for (current_distance = 0; current_distance < listOfDistances.size(); ++current_distance) {
        if (listOfDistances[current_distance] == curNode.radius) {
            break;
        }
    }

    std::vector<circleNode> circle_nodes = circleNodes[current_distance];

    bool successors_are_fine = false;
    auto parent = &(close.find(curNode.convolution(map.getWidth()))->second);
    if (curNode.parent != nullptr) {
        int node_straight_ahead = (int)round(curNode.angle * circleNodes[current_distance].size() / 360) % circleNodes[current_distance].size();
        double angle = fabs(curNode.angle - circleNodes[current_distance][node_straight_ahead].heading);
        if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
            int new_pos_i = curNode.i + circle_nodes[node_straight_ahead].i;
            int new_pos_j = curNode.j + circle_nodes[node_straight_ahead].j;
            if (map.CellOnGrid(new_pos_i, new_pos_j) && map.CellIsTraversable(new_pos_i, new_pos_j)) {
                Node newNode = Node(new_pos_i, new_pos_j);
                newNode.g = curNode.g + getCost(curNode.i, curNode.j, new_pos_i, new_pos_j);
                newNode.angle = circleNodes[current_distance][node_straight_ahead].heading;
                newNode.F = newNode.g + weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j) +
                    curvatureHeuristicWeight * distance * fabs(curNode.angle - newNode.angle);
                newNode.radius = curNode.radius;
                newNode.parent = parent;

                update(curNode, newNode, successors_are_fine, map);
            }
        } // now we will expand neighbors that are closest to the node that lies straight ahead

        std::vector<int> candidates = std::vector<int>{ node_straight_ahead, node_straight_ahead };
        bool limit1 = true;
        bool limit2 = true;
        while (++candidates[0] != --candidates[1] && (limit1 || limit2)) { // untill the whole circle is explored or we exessed anglelimit somewhere
            if (candidates[0] >= circle_nodes.size()) {
                candidates[0] = 0;
            }
            if (candidates[1] < 0) {
                candidates[1] = circle_nodes.size() - 1;
            }

            for (auto cand : candidates) {
                double angle = fabs(curNode.angle - circleNodes[current_distance][cand].heading);
                if ((angle <= 180 && angle <= angleLimit) || (angle > 180 && 360 - angle <= angleLimit)) {
                    int new_pos_i = curNode.i + circle_nodes[cand].i;
                    int new_pos_j = curNode.j + circle_nodes[cand].j;

                    if (!map.CellOnGrid(new_pos_i, new_pos_j) || map.CellIsObstacle(new_pos_i, new_pos_j)) {
                        continue;
                    }

                    Node newNode = Node(new_pos_i, new_pos_j);
                    newNode.g = curNode.g + getCost(curNode.i, curNode.j, new_pos_i, new_pos_j);
                    newNode.angle = circleNodes[current_distance][cand].heading;
                    newNode.F = newNode.g + weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j) +
                        curvatureHeuristicWeight * distance * fabs(curNode.angle - newNode.angle);
                    newNode.radius = curNode.radius;
                    newNode.parent = parent;

                    update(curNode, newNode, successors_are_fine, map);
                }
                else if (cand == candidates[0]) {
                    limit1 = false;
                }
                else {
                    limit2 = false;
                }
            }
        }
    }
    else { // when we do not have parent, we should explore all neighbors
        int angle_position(-1), new_pos_i, new_pos_j;
        for (auto& node : circle_nodes) {
            new_pos_i = curNode.i + node.i;
            new_pos_j = curNode.j + node.j;
            angle_position++;

            if (!map.CellOnGrid(new_pos_i, new_pos_j) || map.CellIsObstacle(new_pos_i, new_pos_j)) {
                continue;
            }

            Node newNode = Node(new_pos_i, new_pos_j);
            newNode.g = curNode.g + getCost(curNode.i, curNode.j, new_pos_i, new_pos_j);
            newNode.F = newNode.g + weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j);
            newNode.radius = curNode.radius;
            newNode.angle = circleNodes[current_distance][angle_position].heading;
            newNode.parent = parent;

            update(curNode, newNode, successors_are_fine, map);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(curNode.i, curNode.j, map.goal_i, map.goal_j) <= curNode.radius) {
        double angle = calcAngle(*curNode.parent, curNode, Node(map.goal_i, map.goal_j));

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= angleLimit) {
            Node newNode = Node(map.goal_i, map.goal_j,
                curNode.g + getCost(curNode.i, curNode.j, map.goal_i, map.goal_j), 0.0,
                curNode.radius, parent, curvatureHeuristicWeight * distance, 0.0);

            update(curNode, newNode, successors_are_fine, map);
        }
    }
    return successors_are_fine;
}


bool LianSearch::tryToDecreaseRadius(Node& curNode, int width) {
    auto radiusIter = std::lower_bound(listOfDistances.rbegin(), listOfDistances.rend(), curNode.radius);
    if (radiusIter == listOfDistances.rbegin()) {
        return false;
    }

    curNode.radius = *(radiusIter - 1);
    auto it = close.find(curNode.convolution(width));
    auto range = close.equal_range(it->first);
    for (auto it = range.first; it != range.second; ++it) {
        if (it->second.parent && it->second.parent->i == curNode.parent->i
            && it->second.parent->j == curNode.parent->j) {
            it->second.radius = curNode.radius;
            break;
        }
    }
    return true;
}

void LianSearch::makePrimaryPath(Node* curNode) {
    while (curNode) {
        hppath.push_front(*curNode);
        curNode = curNode->parent;
    }
}

bool LianSearch::checkAngle(const Node& dad, const Node& node, const Node& son) const {
    double angle = calcAngle(dad, node, son) * 180 / CN_PI_CONSTANT;
    if (fabs(angle) <= angleLimit) {
        return true;
    }
    return false;
}


std::list<Node> LianSearch::smoothPath(const std::list<Node>& path, const Map& map) {
    std::list<Node> new_path;
    sresult.pathLength = 0;
    auto it = path.begin();
    auto curr_it = path.begin();
    Node start_section = path.front();
    Node end_section = path.front();
    bool first = true;
    Node previous = *it++;
    while (end_section != path.back()) {
        for (it; it != path.end(); ++it) {
            auto next = ++it;
            --it;
            if (!first && !checkAngle(previous, start_section, *it)) {
                continue;
            }
            if (((next != path.end() && checkAngle(start_section, *it, *next))
                || next == path.end()) && checkLineSegment(map, start_section, *it)) {
                end_section = *it;
                curr_it = it;
            }
        }
        sresult.pathLength += (double)getCost(previous.i, previous.j, start_section.i, start_section.j);
        new_path.push_back(start_section);
        previous = start_section;
        first = false;
        start_section = end_section;
        it = ++curr_it;
    }
    sresult.pathLength += (double)getCost(previous.i, previous.j, end_section.i, end_section.j);
    new_path.push_back(end_section);
    return new_path;
}

void LianSearch::makeSecondaryPath() {
    std::vector<Node> lineSegment;
    auto it = hppath.begin();
    Node parent = *it++;
    while (it != hppath.end()) {
        calculateLineSegment(lineSegment, parent, *it);
        std::reverse(std::begin(lineSegment), std::end(lineSegment));
        lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
        parent = *it++;
    }
    lppath.push_front(hppath.back());
    std::reverse(std::begin(lppath), std::end(lppath));
}


double LianSearch::makeAngles() {
    angles.clear();
    double max_angle = 0;
    sresult.accumAngle = 0;
    auto pred = hppath.begin();
    auto current = ++hppath.begin();
    auto succ = ++(++hppath.begin());

    while (succ != hppath.end()) {
        double angle = calcAngle(*pred++, *current++, *succ++);
        angle = angle * 180 / CN_PI_CONSTANT;
        if (angle > max_angle) max_angle = angle;
        sresult.accumAngle += angle;
        angles.push_back(angle);
    }
    std::reverse(std::begin(angles), std::end(angles));
    return max_angle;
}
