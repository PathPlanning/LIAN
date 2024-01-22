#include "liansearch.h"

#include <cmath>

double getCost(int a_i, int a_j, int b_i, int b_j) {
    return sqrt(abs(a_i - b_i) * abs(a_i - b_i) + abs(a_j - b_j) * abs(a_j - b_j));
}

void saveIterationToLog(std::shared_ptr<Logger> &logger, int closeSize_, const Node& curNode) {
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

/*
 * // Use for more accurate time calculation
 * #ifdef __linux__
 *     #include <sys/time.h>
 * #else
 *     #include <windows.h>
 * #endif
 *
*/

std::vector<int> LianSearch::buildDistances() const {
    std::vector<int> result;
    int curDistance = settings_.distance;
    if (settings_.decreaseDistanceFactor > 1) {
        while (curDistance >= settings_.distanceMin) {
            result.push_back(curDistance);
            curDistance = std::ceil((float)curDistance / settings_.decreaseDistanceFactor);
        }
    }
    else {
        result.push_back(curDistance);
    }
    return result;
}

LianSearch::LianSearch(SearchParams settings)
    : settings_(settings)
    , distanceLookup_(buildDistances())
{
    srand(time(nullptr));
}

void LianSearch::calculateCircle(int radius) { //here radius - radius of the circle in cells
    circleNodes_.clear();
    circleNodes_.resize(distanceLookup_.size());
    for (int k = 0; k < distanceLookup_.size(); ++k) {
        radius = distanceLookup_[k];
        circleNodes_[k].clear();
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
            circleNodes_[k].push_back(circle_nodes[i]);
        }
        for (int i = circle_nodes.size() - 7; i > 0; i -= 4) {
            circleNodes_[k].push_back(circle_nodes[i]);
        }
        for (int i = 7; i < circle_nodes.size(); i += 4) {
            circleNodes_[k].push_back(circle_nodes[i]);
        }
        for (int i = circle_nodes.size() - 6; i > 0; i -= 4) {
            circleNodes_[k].push_back(circle_nodes[i]);
        }
        circleNodes_[k].pop_back();
        for (size_t i = 0; i < circleNodes_[k].size(); ++i) {
            double angle = acos((circleNodes_[k][0].i * circleNodes_[k][i].i + circleNodes_[k][0].j * circleNodes_[k][i].j)
                / (sqrt(pow(circleNodes_[k][0].i, 2) + pow(circleNodes_[k][0].j, 2))
                    * sqrt(pow(circleNodes_[k][i].i, 2) + pow(circleNodes_[k][i].j, 2))));
            if (i < circleNodes_[k].size() / 2) {
                circleNodes_[k][i].heading = angle * 180 / CN_PI_CONSTANT;
            }
            else {
                circleNodes_[k][i].heading = 360 - angle * 180 / CN_PI_CONSTANT;
            }
            //std::cout << circleNodes[k][i].heading  << std::endl;
        }
    }
}

void LianSearch::calculatePivotCircle() {
    pivotCircle_.clear();
    int add_i;
    int add_j;
    int num = floor(settings_.pivotRadius + 0.5 - CN_EPSILON);
    for (int i = -num; i <= +num; i++) {
        for (int j = -num; j <= +num; j++) {
            add_i = i != 0 ? 1 : 0;
            add_j = j != 0 ? 1 : 0;
            if ((pow(2 * abs(i) - add_i, 2) + pow(2 * abs(j) - add_j, 2)) < pow(2 * settings_.pivotRadius, 2)) {
                pivotCircle_.emplace_back( i, j );
            }
        }
    }
    if (pivotCircle_.empty()) {
        pivotCircle_.emplace_back( 0, 0 );
    }
}

bool LianSearch::checkPivotCircle(const Map& map, const Node& center) {
    int i, j;
    for (auto & cell : pivotCircle_) {
        i = center.i + cell.first;
        j = center.j + cell.second;
        if (!map.CellOnGrid(i, j) || map.CellIsObstacle(i, j)) {
            return false;
        }
    }
    return true;
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

std::vector<Node> LianSearch::calculateLineSegment(const Node& start, const Node& goal) {
    std::vector<Node> line;
    bool reversed = lineSegmentTraverse(start, goal,
        [&line](int64_t t, int64_t c) {
            line.emplace_back(t, c);
            return true;
        });
    if (reversed) {
        std::reverse(line.begin(), line.end());
    }
    return line;
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
    if (search_tree_.empty()) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }

    if (search_tree_.sizeClose() > settings_.stepLimit && settings_.stepLimit > 0) {
        std::cout << "Algorithm exceeded step limit!" << std::endl;
        return true;
    }
    return false;
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

SearchResult LianSearch::startSearch(std::shared_ptr<Logger> logger, const Map& map) {
    Node curNode(map.start_i, map.start_j, 0.0, 0.0, 0.0);
    curNode.radius = settings_.distance;
    curNode.F = settings_.weight * getCost(curNode.i, curNode.j, map.goal_i, map.goal_j);
    bool pathFound = false;
    search_tree_.addOpen(curNode);
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
        std::optional<Node> min = search_tree_.getOpen();
        if (!min)
            break;
        curNode = *min;
        search_tree_.addClose( curNode);

        saveIterationToLog(logger, search_tree_.sizeClose(), curNode);

        if (curNode.i == map.goal_i && curNode.j == map.goal_j) { // if current point is goal point - end of the cycle
            pathFound = true;
            break;
        }

        if (!expand(curNode, map) && distanceLookup_.size() > 1)
            while (curNode.radius > distanceLookup_[distanceLookup_.size() - 1])
                if (tryToDecreaseRadius(curNode))
                    if (expand(curNode, map))
                        break;

        search_tree_.saveToLogOpenAndClose(logger);
    }

    search_tree_.saveToLogOpenAndClose(logger);

    sresult_.nodesCreated = search_tree_.sizeOpen() + search_tree_.sizeClose();
    sresult_.numberOfSteps = search_tree_.sizeClose();
    if (pathFound) {
        sresult_.pathLength = curNode.g;
        makePrimaryPath(&curNode);
        if (settings_.postsmoother) {
            hppath_ = smoothPath(hppath_, map);
        }
        makeSecondaryPath();
        float max_angle = makeAngles();
        sresult_.pathFound = true;
        sresult_.hpPath = hppath_;
        sresult_.lpPath = lppath_;
        sresult_.angles = angles_;
        sresult_.maxAngle = max_angle;
        sresult_.sections = hppath_.size() - 1;

        end = std::chrono::system_clock::now();
        sresult_.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;
        /* // for more accurate time calculation
       #ifdef __linux__
           gettimeofday(&end, NULL);
           sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
       #else
           QueryPerformanceCounter(&end);
           sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
       #endif */

        return sresult_;
    }
    else {
        sresult_.pathFound = false;

        end = std::chrono::system_clock::now();
        sresult_.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;

        /* for more accurate time calculation
       #ifdef __linux__
           gettimeofday(&end, NULL);
           sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
       #else
           QueryPerformanceCounter(&end);
           sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
       #endif */

        return sresult_;
    }
}

void LianSearch::update(const Node& current_node, Node &new_node, bool& successors, const Map& map) {
    if (!checkLineSegment(map, *new_node.parent, new_node)) {
        return;
    }
    if (settings_.pivotRadius > 0 && (new_node.i != map.goal_i || new_node.j != map.goal_j)) {
        if (!checkPivotCircle(map, new_node)) {
            return;
        }
    }

    auto range = search_tree_.findCloseRange(new_node);
    for (auto it = range.first; it != range.second; ++it) {
        Node *parent = it->second.parent;
        if (parent == nullptr || *parent == current_node) {
            return;
        }
    }

    if (distanceLookup_.size() > 1) {
        new_node.radius = tryToIncreaseRadius(&new_node);
    }
    search_tree_.addOpen(new_node);
    successors = true;
}

bool LianSearch::expand(const Node& curNode, const Map& map) {
    std::size_t current_distance;
    for (current_distance = 0; current_distance < distanceLookup_.size(); ++current_distance) {
        if (distanceLookup_[current_distance] == curNode.radius) {
            break;
        }
    }

    std::vector<circleNode> circle_nodes = circleNodes_[current_distance];
    
    bool successors_are_fine = false;
    Node *curNodeFromClose = search_tree_.findClose(curNode);
    auto tryUpdateMap = [&](auto& node, bool updateAngle = false, bool usesCurvatureHeuristicWeight = true) {
        double angle = fabs(curNode.angle - node.heading);
        if (!(angle <= 180 && angle <= settings_.angleLimit) && !(angle > 180 && 360 - angle <= settings_.angleLimit)) {
            return false; // this is angle problem
        }

        int new_pos_i = curNode.i + node.i;
        int new_pos_j = curNode.j + node.j;

        if (!map.CellOnGrid(new_pos_i, new_pos_j) || map.CellIsObstacle(new_pos_i, new_pos_j)) {
            return true; // this is okay
        }
        Node newNode = Node(new_pos_i, new_pos_j);
        newNode.g = curNode.g + getCost(curNode.i, curNode.j, new_pos_i, new_pos_j);
        newNode.angle = updateAngle ? node.heading : 0.0;
        newNode.F = newNode.g + settings_.weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j) +
            (usesCurvatureHeuristicWeight ?
                settings_.curvatureHeuristicWeight * (float)settings_.distance * fabs(curNode.angle - newNode.angle) : 0.0);
        newNode.radius = curNode.radius;
        newNode.angle = node.heading;
        newNode.parent = curNodeFromClose;

        update(curNode, newNode, successors_are_fine, map);
        return true; // this is okey
        };

    if (curNode.parent != nullptr) {
        int node_straight_ahead = (int)round(curNode.angle * circle_nodes.size() / 360) % circle_nodes.size();
        tryUpdateMap(circle_nodes[node_straight_ahead], true); // now we will expand neighbors that are closest to the node that lies straight ahead

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

            for (auto &cand : candidates) {
                if (tryUpdateMap(circle_nodes[cand])) {
                    continue;
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
            newNode.F = newNode.g + settings_.weight * getCost(new_pos_i, new_pos_j, map.goal_i, map.goal_j);
            newNode.radius = curNode.radius;
            newNode.angle = circle_nodes[angle_position].heading;
            newNode.parent = curNodeFromClose;

            update(curNode, newNode, successors_are_fine, map);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(curNode.i, curNode.j, map.goal_i, map.goal_j) <= curNode.radius) {
        double angle = curNode.parent != nullptr ? calcAngle(*curNode.parent, curNode, Node(map.goal_i, map.goal_j)) : 0.0;

        if (fabs(angle * 180 / CN_PI_CONSTANT) <= settings_.angleLimit) {
            Node newNode = Node(map.goal_i, map.goal_j,
                curNode.g + getCost(curNode.i, curNode.j, map.goal_i, map.goal_j), 0.0,
                                curNode.radius, curNodeFromClose, settings_.curvatureHeuristicWeight * settings_.distance, 0.0);

            update(curNode, newNode, successors_are_fine, map);
        }
    }
    return successors_are_fine;
}

int LianSearch::tryToIncreaseRadius(Node* curNode) {
    std::size_t k = 0;
    while (k < settings_.numOfParentsToIncreaseRadius) {
        if (!(curNode->parent != nullptr && curNode->radius == curNode->parent->radius)) {
            break;
        }
        ++k;
        curNode = curNode->parent;
    }
    std::ptrdiff_t i;
    bool change = false;
    if (k == settings_.numOfParentsToIncreaseRadius) {
        for (i = distanceLookup_.size() - 1; i >= 0; --i) {
            if (curNode->radius == distanceLookup_[i]) {
                break;
            }
        }
        change = i > 0;
    }
    if (change) {
        return distanceLookup_[i - 1];
    }
    return curNode->radius;
}

bool LianSearch::tryToDecreaseRadius(Node& curNode) {
    auto radiusIter = std::lower_bound(distanceLookup_.rbegin(), distanceLookup_.rend(), curNode.radius);
    if (radiusIter == distanceLookup_.rbegin()) {
        return false;
    }

    curNode.radius = *(radiusIter - 1);
    auto range = search_tree_.findCloseRange(curNode);
    for (auto &it = range.first; it != range.second; ++it) {
        Node &nodeFromClose = it->second;
        if (areFromSameSource(curNode, nodeFromClose)) {
            nodeFromClose.radius = curNode.radius;
            break;
        }
    }
    return true;
}

void LianSearch::makePrimaryPath(Node* curNode) {
    while (curNode) {
        hppath_.push_front(*curNode);
        curNode = curNode->parent;
    }
}

bool LianSearch::checkAngle(const Node& dad, const Node& node, const Node& son) const {
    double angle = calcAngle(dad, node, son) * 180 / CN_PI_CONSTANT;
    if (fabs(angle) <= settings_.angleLimit) {
        return true;
    }
    return false;
}


std::list<Node> LianSearch::smoothPath(const std::list<Node>& path, const Map& map) {
    std::list<Node> new_path;
    sresult_.pathLength = 0;
    auto it = path.begin();
    auto curr_it = path.begin();
    Node start_section = path.front();
    Node end_section = path.front();
    bool first = true;
    Node previous = *it++;
    while (end_section != path.back()) {
        for (it; it != path.end(); ++it) {
            auto& next = ++it;
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
        sresult_.pathLength += (double)getCost(previous.i, previous.j, start_section.i, start_section.j);
        new_path.push_back(start_section);
        previous = start_section;
        first = false;
        start_section = end_section;
        it = ++curr_it;
    }
    sresult_.pathLength += (double)getCost(previous.i, previous.j, end_section.i, end_section.j);
    new_path.push_back(end_section);
    return new_path;
}

void LianSearch::makeSecondaryPath() {
    ;
    auto it = hppath_.begin();
    Node parent = *it++;
    while (it != hppath_.end()) {
        std::vector<Node> lineSegment = calculateLineSegment(parent, *it);
        std::reverse(std::begin(lineSegment), std::end(lineSegment));
        lppath_.insert(lppath_.begin(), ++lineSegment.begin(), lineSegment.end());
        parent = *it++;
    }
    lppath_.push_front(hppath_.back());
    std::reverse(std::begin(lppath_), std::end(lppath_));
}


double LianSearch::makeAngles() {
    angles_.clear();
    double max_angle = 0;
    sresult_.accumAngle = 0;
    auto pred = hppath_.begin();
    auto current = ++hppath_.begin();
    auto succ = ++(++hppath_.begin());

    while (succ != hppath_.end()) {
        double angle = calcAngle(*pred++, *current++, *succ++);
        angle = angle * 180 / CN_PI_CONSTANT;
        if (angle > max_angle) max_angle = angle;
        sresult_.accumAngle += angle;
        angles_.push_back(angle);
    }
    std::reverse(std::begin(angles_), std::end(angles_));
    return max_angle;
}
