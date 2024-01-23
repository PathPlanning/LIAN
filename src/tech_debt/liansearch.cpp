#include "liansearch.h"

#include <numbers>
#include <cmath>

double getCost(int a_i, int a_j, int b_i, int b_j) {
    return sqrt(abs(a_i - b_i) * abs(a_i - b_i) + abs(a_j - b_j) * abs(a_j - b_j));
}

void saveIterationToLog(std::shared_ptr<Logger> &logger, int closeSize_, const Node& curNode) {
    auto space = logger->logSpace<Logger::Levels::iter>(Logger::Tags::iterations);
    if (!space) {
        return;
    }

    TiXmlElement element(Logger::Tags::step);

    element.SetAttribute(Logger::Tags::step, closeSize_);
    element.SetAttribute(Logger::Tags::parentX, curNode.j);
    element.SetAttribute(Logger::Tags::y, curNode.i);
    if (curNode.parent) {
        element.SetAttribute(Logger::Tags::parentX, curNode.parent->j);
        element.SetAttribute(Logger::Tags::parentY, curNode.parent->i);
    }
    element.SetDoubleAttribute(Logger::Tags::F, curNode.F);
    element.SetDoubleAttribute(Logger::Tags::g, curNode.g);

    space->InsertEndChild(element);
}

void saveToLogOpenAndClose(std::shared_ptr<Logger> logger, const OpenList &open_,
    const std::unordered_multimap<int, Node> &close_) {
    auto space = logger->logSpace<Logger::Levels::low>(Logger::Tags::lowLevel);
    if (!space) {
        return;
    }

    int iterate = 0;
    TiXmlNode *child = nullptr, *curNode = space;

    while (child = curNode->IterateChildren(child)) {
        iterate++;
    }

    {
        TiXmlElement element(Logger::Tags::step);
        element.SetAttribute(Logger::Tags::number, iterate);
        curNode->InsertEndChild(element);
        curNode = curNode->LastChild();
    }

    {

        TiXmlElement element(Logger::Tags::open);
        curNode->InsertEndChild(element);
        child = curNode->LastChild();
    }

    open_.writeToXml(child);

    {
        TiXmlElement element(Logger::Tags::close);
        curNode->InsertEndChild(element);
        child = curNode->LastChild();
    }

    for (const auto &it : close_) {
        TiXmlElement element(Logger::Tags::node);
        element.SetAttribute(Logger::Tags::x, it.second.j);
        element.SetAttribute(Logger::Tags::y, it.second.i);
        element.SetDoubleAttribute(Logger::Tags::F, it.second.F);
        element.SetDoubleAttribute(Logger::Tags::g, it.second.g);
        if (it.second.g > 0) {
            element.SetAttribute(Logger::Tags::parentX, it.second.parent->j);
            element.SetAttribute(Logger::Tags::parentY, it.second.parent->i);
        }
        child->InsertEndChild(element);
    }
}

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
        auto &circleNodesK = circleNodes_[k];
        circleNodesK.clear();
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
            circleNodesK.push_back(circle_nodes[i]);
        }
        for (int i = circle_nodes.size() - 7; i > 0; i -= 4) {
            circleNodesK.push_back(circle_nodes[i]);
        }
        for (int i = 7; i < circle_nodes.size(); i += 4) {
            circleNodesK.push_back(circle_nodes[i]);
        }
        for (int i = circle_nodes.size() - 6; i > 0; i -= 4) {
            circleNodesK.push_back(circle_nodes[i]);
        }
        circleNodesK.pop_back();
        auto &start = circleNodesK[0];
        for (auto &node : circleNodesK) {
            double angle = acos((start.i * node.i + start.j * node.j)
                / (sqrt(pow(start.i, 2) + pow(start.j, 2))
                    * sqrt(pow(node.i, 2) + pow(node.j, 2))));
            if (node.i >= 0) {
                node.heading = angle * 180 / std::numbers::pi;
            }
            else {
                node.heading = 360 - angle * 180 / std::numbers::pi;
            }
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
    if (open_.size() == 0) {
        std::cout << "OPEN list is empty!" << std::endl;
        return true;
    }

    if (close_.size() > settings_.stepLimit && settings_.stepLimit > 0) {
        std::cout << "Algorithm esceeded step limit!" << std::endl;
        return true;
    }
    return false;
}


double LianSearch::calcAngle(const Node& dad, const Node& node, const Node& son) {
    double cos_angle = (node.j - dad.j) * (son.j - node.j) +
        (node.i - dad.i) * (son.i - node.i);
    cos_angle /= getCost(son.i, son.j, node.i, node.j);
    cos_angle /= getCost(node.i, node.j, dad.i, dad.j);

    cos_angle = std::min(cos_angle, 1.0);
    cos_angle = std::max(cos_angle, -1.0);

    return acos(cos_angle);
}

SearchResult LianSearch::startSearch(std::shared_ptr<Logger> logger, const Map& map) {
    open_.resize(map.getHeight());
    Node curNode(map.start_i, map.start_j, 0.0, 0.0, 0.0);
    curNode.radius = settings_.distance;
    curNode.F = settings_.weight * getCost(curNode.i, curNode.j, map.goal_i, map.goal_j);
    bool pathFound = false;
    open_.add(curNode);
    calculateCircle((int)curNode.radius);
    calculatePivotCircle();

    std::chrono::time_point<std::chrono::system_clock> begin, end;
    begin = std::chrono::system_clock::now();

    while (!stopCriterion()) { // main cycle of the search
        curNode = open_.getMin();
        close_.insert({ curNode.convolution(map.getWidth()),curNode });

        saveIterationToLog(logger, close_.size(), curNode);

        if (curNode.i == map.goal_i && curNode.j == map.goal_j) { // if current point is goal point - end of the cycle
            pathFound = true;
            break;
        }

        if (!expand(curNode, map) && distanceLookup_.size() > 1)
            while (curNode.radius > distanceLookup_[distanceLookup_.size() - 1])
                if (tryToDecreaseRadius(curNode, map.getWidth()))
                    if (expand(curNode, map))
                        break;

        saveToLogOpenAndClose(logger, open_, close_);
    }

    saveToLogOpenAndClose(logger, open_, close_);

    sresult_.nodesCreated = open_.size() + close_.size();
    sresult_.numberOfSteps = close_.size();
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

        return sresult_;
    }
    else {
        sresult_.pathFound = false;

        end = std::chrono::system_clock::now();
        sresult_.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) / 1000000000;

        return sresult_;
    }
}

void LianSearch::updateOpen(const Node& current_node, Node &new_node, bool& successors, const Map& map) {
    if (!checkLineSegment(map, *new_node.parent, new_node)) {
        return;
    }
    if (settings_.pivotRadius > 0 && (new_node.i != map.goal_i || new_node.j != map.goal_j)) {
        if (!checkPivotCircle(map, new_node)) {
            return;
        }
    }

    auto it = close_.find(new_node.convolution(map.getWidth()));
    if (it != close_.end()) {
        auto range = close_.equal_range(it->first);
        for (auto it = range.first; it != range.second; ++it) {
            if (it->second.parent == nullptr) {
                return;
            }
            if (it->second.parent->i == current_node.i && it->second.parent->j == current_node.j) {
                return;
            }
        }
    }

    if (distanceLookup_.size() > 1) {
        new_node.radius = tryToIncreaseRadius(&new_node);
    }
    open_.add(new_node);
    successors = true;
}

bool LianSearch::expand(const Node& curNode, const Map& map) {
    auto radiusIter = std::upper_bound(distanceLookup_.rbegin(), distanceLookup_.rend(), curNode.radius);
    std::vector<circleNode> circle_nodes = circleNodes_[distanceLookup_.rend() - radiusIter];
    
    bool successors_are_fine = false;
    auto parent = &(close_.find(curNode.convolution(map.getWidth()))->second);
    auto tryUpdateOpen = [&](auto& node, bool updateAngle = false, bool usesCurvatureHeuristicWeight = true) {
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
        newNode.parent = parent;

        updateOpen(curNode, newNode, successors_are_fine, map);
        return true; // this is okay
        };

    if (curNode.parent != nullptr) {
        int node_straight_ahead = (int)round(curNode.angle * circle_nodes.size() / 360) % circle_nodes.size();
        tryUpdateOpen(circle_nodes[node_straight_ahead], true); // now we will expand neighbors that are closest to the node that lies straight ahead

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
                if (tryUpdateOpen(circle_nodes[cand])) {
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
        for (auto& node : circle_nodes) {
            tryUpdateOpen(node);
        }
    }

    // when we are near goal point, we should try to reach it
    if (getCost(curNode.i, curNode.j, map.goal_i, map.goal_j) <= curNode.radius) {
        double angle = curNode.parent ? calcAngle(*curNode.parent, curNode, Node(map.goal_i, map.goal_j)) : 0.0;

        if (fabs(angle * 180 / std::numbers::pi) <= settings_.angleLimit) {
            Node newNode = Node(map.goal_i, map.goal_j,
                curNode.g + getCost(curNode.i, curNode.j, map.goal_i, map.goal_j), 0.0,
                curNode.radius, parent, settings_.curvatureHeuristicWeight * settings_.distance, 0.0);

            updateOpen(curNode, newNode, successors_are_fine, map);
        }
    }
    return successors_are_fine;
}

int LianSearch::tryToIncreaseRadius(Node *curNode) {
    std::size_t k = 0;
    while (k < settings_.numOfParentsToIncreaseRadius) {
        if (!(curNode->parent && curNode->radius == curNode->parent->radius)) {
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

bool LianSearch::tryToDecreaseRadius(Node &curNode, int width) {
    auto radiusIter = std::lower_bound(distanceLookup_.rbegin(), distanceLookup_.rend(), curNode.radius);
    if (radiusIter == distanceLookup_.rbegin()) {
        return false;
    }

    curNode.radius = *(radiusIter - 1);
    auto it = close_.find(curNode.convolution(width));
    auto range = close_.equal_range(it->first);
    for (auto it = range.first; it != range.second; ++it) {
        if (it->second.parent && it->second.parent->i == curNode.parent->i
            && it->second.parent->j == curNode.parent->j) {
            it->second.radius = curNode.radius;
            break;
        }
    }
    return true;
}

void LianSearch::makePrimaryPath(Node *curNode) {
    while (curNode) {
        hppath_.push_front(*curNode);
        curNode = curNode->parent;
    }
}

bool LianSearch::checkAngle(const Node &dad, const Node &node, const Node &son) const {
    double angle = calcAngle(dad, node, son) * 180 / std::numbers::pi;
    return fabs(angle) <= settings_.angleLimit;
}


std::list<Node> LianSearch::smoothPath(const std::list<Node> &path, const Map &map) {
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
            auto &next = ++it;
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
        angle = angle * 180 / std::numbers::pi;
        if (angle > max_angle) max_angle = angle;
        sresult_.accumAngle += angle;
        angles_.push_back(angle);
    }
    std::reverse(std::begin(angles_), std::end(angles_));
    return max_angle;
}
