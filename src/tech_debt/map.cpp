#include "xml_utils.h"
#include "map.h"

namespace {
    constexpr auto tagRoot = "root";
    constexpr auto tagMap = "map";
    constexpr auto tagHeight = "height";
    constexpr auto tagWidth = "width";
    constexpr auto tagStartX = "startx";
    constexpr auto tagStartY = "starty";
    constexpr auto tagFinishX = "finishx";
    constexpr auto tagFinishY = "finishy";
    constexpr auto tagCellSize = "cellsize";
    constexpr auto tagGrid = "grid";
    constexpr auto tagRow = "row";
    constexpr auto tagObstacle = 1;
}

std::vector<int>& Map::operator[](int i) {
    return grid_[i];
}
const std::vector<int>& Map::operator[](int i) const {
    return grid_[i];
}

bool Map::CellIsTraversable(int curr_i, int curr_j) const {
    return (grid_[curr_i][curr_j] != tagObstacle);
}

bool Map::CellIsObstacle(int curr_i, int curr_j) const {
    return (grid_[curr_i][curr_j] == tagObstacle);
}

bool Map::CellOnGrid(int curr_i, int curr_j) const {
    return (curr_i < height_ && curr_i >= 0 && curr_j < width_ && curr_j >= 0);
}

int Map::getHeight() const {
    return height_;
}

int Map::getWidth() const {
    return width_;
}

double Map::getCellSize() const {
    return cellSize_;
}

Map::Map(const char* FileName) {
    TiXmlDocument doc(FileName);
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file");
    }

    auto root = doc.FirstChildElement(tagRoot);
    if (!root) {
        std::ostringstream err_oss;
        err_oss << "Error! No '" << tagRoot << "' element found in XML file";
        throw std::runtime_error(err_oss.str());
    }

    TiXmlElement *map = getElement(root, tagMap, tagRoot);

    auto element = getElement(map, tagHeight, tagMap);
    height_ = std::abs(serialize<int>(element));

    element = getElement(map, tagWidth, tagMap);
    width_ = std::abs(serialize<int>(element));

    element = getElement(map, tagCellSize, tagMap);
    cellSize_ = std::abs(serialize<int>(element));


    element = getElement(map, tagStartX, tagMap);
    start_j = serialize<int>(element);

    element = getElement(map, tagStartY, tagMap);
    start_i = serialize<int>(element);

    element = getElement(map, tagFinishX, tagMap);
    goal_j = serialize<int>(element);

    element = getElement(map, tagFinishY, tagMap);
    goal_i = serialize<int>(element);

    element = getElement(map, tagGrid, tagMap);
    grid_.reserve(height_);

    element = element->FirstChildElement(tagRow);

    for (int curY = 0; curY < height_; ++curY) {
        if (!element) {
            std::ostringstream err_oss;
            err_oss << "Not enough '" << tagRow << "' in '" << tagGrid << "' given.";
            throw std::runtime_error(err_oss.str());
        }

        grid_.push_back(serializeVector<int>(element));
        if (grid_.back().size() != width_) {
            std::ostringstream err_oss;
            err_oss << "Wrong amount of cells in '" << tagRow << "' " << curY << " given.";
            throw std::runtime_error(err_oss.str());
        }
        element = element->NextSiblingElement();
    }
}
