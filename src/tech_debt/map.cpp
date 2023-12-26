#include "xmlUtils.h"
#include "map.h"

std::vector<int>& Map::operator[](int i) {
    return grid_[i];
}
const std::vector<int>& Map::operator[](int i) const {
    return grid_[i];
}

bool Map::CellIsTraversable(int curr_i, int curr_j) const {
    return (grid_[curr_i][curr_j] != CN_OBSTL);
}

bool Map::CellIsObstacle(int curr_i, int curr_j) const {
    return (grid_[curr_i][curr_j] == CN_OBSTL);
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

    auto root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::ostringstream err_oss;
        err_oss << "Error! No '" << CNS_TAG_ROOT << "' element found in XML file";
        throw std::runtime_error(err_oss.str());
    }

    TiXmlElement *map = getElement(root, CNS_TAG_MAP, CNS_TAG_ROOT);

    auto element = getElement(map, CNS_TAG_HEIGHT, CNS_TAG_MAP);
    height_ = std::abs(serialize<int>(element));

    element = getElement(map, CNS_TAG_WIDTH, CNS_TAG_MAP);
    width_ = std::abs(serialize<int>(element));

    element = getElement(map, CNS_TAG_CELLSIZE, CNS_TAG_MAP);
    cellSize_ = std::abs(serialize<int>(element));


    element = getElement(map, CNS_TAG_SX, CNS_TAG_MAP);
    start_j = serialize<int>(element);

    element = getElement(map, CNS_TAG_SY, CNS_TAG_MAP);
    start_i = serialize<int>(element);

    element = getElement(map, CNS_TAG_FX, CNS_TAG_MAP);
    goal_j = serialize<int>(element);

    element = getElement(map, CNS_TAG_FY, CNS_TAG_MAP);
    goal_i = serialize<int>(element);

    element = getElement(map, CNS_TAG_GRID, CNS_TAG_MAP);
    grid_.reserve(height_);

    element = element->FirstChildElement(CNS_TAG_ROW);

    for (int curY = 0; curY < height_; ++curY) {
        if (!element) {
            std::ostringstream err_oss;
            err_oss << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given.";
            throw std::runtime_error(err_oss.str());
        }

        grid_.push_back(serializeVector<int>(element));
        if (grid_.back().size() != width_) {
            std::ostringstream err_oss;
            err_oss << "Wrong amount of cells in '" << CNS_TAG_ROW << "' " << curY << " given.";
            throw std::runtime_error(err_oss.str());
        }
        element = element->NextSiblingElement();
    }
}
