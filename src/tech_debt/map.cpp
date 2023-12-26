#include "xmlUtils.h"
#include "map.h"

Map::~Map()
{	
    if(Grid) {
        for(int i = 0; i < height; i++) {
            delete[] Grid[i];
        }
        delete[] Grid;
    }
}

int * Map::operator [] (int i) {
    return Grid[i];
}
const int * Map::operator [] (int i) const {
    return Grid[i];
}

bool Map::CellIsTraversable(int curr_i, int curr_j) const {
    return (Grid[curr_i][curr_j] != CN_OBSTL);
}

bool Map::CellIsObstacle(int curr_i, int curr_j) const {
    return (Grid[curr_i][curr_j] == CN_OBSTL);
}

bool Map::CellOnGrid(int curr_i, int curr_j) const {
    return (curr_i < height && curr_i >= 0 && curr_j < width && curr_j >= 0);
}

int Map::getHeight() const {
    return height;
}

int Map::getWidth() const {
    return width;
}

double Map::getCellSize() const {
    return CellSize;
}

Map::Map(const char* FileName) : Grid(nullptr) {
    TiXmlDocument doc(FileName);
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file");
    }

    auto root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        throw std::runtime_error((std::stringstream() << "Error! No '" << CNS_TAG_ROOT << "' element found in XML file").str());
    }

    TiXmlElement *map = getElement(root, CNS_TAG_MAP, CNS_TAG_ROOT);

    auto element = getElement(map, CNS_TAG_HEIGHT, CNS_TAG_MAP);
    height = std::abs(serialize<int>(element));

    element = getElement(map, CNS_TAG_WIDTH, CNS_TAG_MAP);
    width = std::abs(serialize<int>(element));

    element = getElement(map, CNS_TAG_CELLSIZE, CNS_TAG_MAP);
    CellSize = std::abs(serialize<int>(element));


    element = getElement(map, CNS_TAG_SX, CNS_TAG_MAP);
    start_j = serialize<int>(element);

    element = getElement(map, CNS_TAG_SY, CNS_TAG_MAP);
    start_i = serialize<int>(element);

    element = getElement(map, CNS_TAG_FX, CNS_TAG_MAP);
    goal_j = serialize<int>(element);

    element = getElement(map, CNS_TAG_FY, CNS_TAG_MAP);
    goal_i = serialize<int>(element);

    element = getElement(map, CNS_TAG_GRID, CNS_TAG_MAP);
    Grid = new int* [height];
    for (int i = 0; i < height; i++) {
        Grid[i] = new int[width];
    }

    element = element->FirstChildElement(CNS_TAG_ROW);

    int i = 0;
    while (i < height) {
        if (!element) {
            throw std::runtime_error((std::stringstream() << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given").str());
        }

        auto grid = element->GetText();
        std::stringstream stream;
        int k = 0;
        std::string text = "";
        int j = 0;

        for (k = 0; k < (strlen(grid)); k++) {
            if (grid[k] == ' ') {
                stream << text;
                stream >> Grid[i][j];
                stream.clear();
                stream.str("");
                text = "";
                j++;
            }
            else {
                text += grid[k];
            }
        }
        stream << text;
        stream >> Grid[i][j];
        stream.clear();
        stream.str("");

        if (j < width - 1) {
            throw std::runtime_error((std::stringstream() << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given").str());
        }
        i++;
        element = element->NextSiblingElement();
    }
}
