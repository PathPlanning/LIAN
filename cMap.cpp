#include"cMap.h"
#include<string>
#include<algorithm>
#include <sstream>

cMap::cMap()
{
    height = -1;
    width = -1;
    start_i = -1;
    start_j = -1;
    goal_i = -1;
    goal_j = -1;
    Grid = 0;
}
cMap::~cMap()
{	
    if(Grid)
    {
        for(int i = 0; i < height; i++)
        {
            delete[] Grid[i];
        }

        delete[] Grid;
    }
}

bool cMap::getMap(const char* FileName)
{
    const char* grid = 0;
    std::string value;
    TiXmlElement *root = 0;
    std::string text = "";
    bool hasGrid = false;
    std::stringstream stream;
    TiXmlDocument doc(FileName);
    if(!doc.LoadFile())
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }
    else

    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML file."<<std::endl;
        return false;
    }

    TiXmlElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    TiXmlNode *node = 0;
    TiXmlElement *element = 0;

    node = map->FirstChild();

    while(node)
    {
        element = node->ToElement();
        value = node->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        if(!hasGrid && height > 0 && width > 0)
        {
            Grid = new int*[height];
            for(int i = 0; i < height; i++)
            {
                Grid[i] = new int[width];
            }

            hasGrid = true;
        }

        if(value == CNS_TAG_HEIGHT)
        {
            text = element->GetText();
            stream<<text;
            stream>>height;
            stream.clear();
            stream.str("");
            if (height <= 0)
            {
                std::cout << "Wrong '"<<CNS_TAG_HEIGHT<<"' value."<<std::endl;
                return false;
            }
        }
        else if(value == CNS_TAG_WIDTH)
        {
            text = element->GetText();
            stream<<text;
            stream>>width;
            stream.clear();
            stream.str("");

            if (width <= 0)
            {
                std::cout << "Wrong '"<<CNS_TAG_WIDTH<<"' value."<<std::endl;
                return false;
            }
        }
        else if(value == CNS_TAG_SX)
        {
            text = element->GetText();
            stream<<text;
            stream>>start_j;
            stream.clear();
            stream.str("");

            if (start_j < 0 || start_j >= width)
            {
                std::cout << "Wrong '"<<CNS_TAG_SX<<"' value."<<std::endl;
                return false;
            }
        }
        else if(value == CNS_TAG_SY)
        {
            text = element->GetText();
            stream<<text;
            stream>>start_i;
            stream.clear();
            stream.str("");

            if (start_i < 0 || start_i >= height)
            {
                std::cout << "Wrong '"<<CNS_TAG_SY<<"' value."<<std::endl;
                return false;
            }
        }
        else if(value == CNS_TAG_FX)
        {
            text = element->GetText();
            stream<<text;
            stream>>goal_j;
            stream.clear();
            stream.str("");

            if (goal_j < 0 || goal_j >= width)
            {
                std::cout << "Wrong '"<<CNS_TAG_FX<<"' value."<<std::endl;
                return false;
            }
        }
        else if(value == CNS_TAG_FY)
        {
            text=element->GetText();
            stream<<text;
            stream>>goal_i;
            stream.clear();
            stream.str("");

            if (goal_i < 0 || goal_i >= height)
            {
                std::cout << "Wrong '"<<CNS_TAG_FY<<"' value."<<std::endl;
                return false;
            }
        }
        else if(value == CNS_TAG_GRID)
        {
            if(height == -1 || width == -1)
            {
                std::cout << "No '"<<CNS_TAG_HEIGHT<<"' or '"<<CNS_TAG_WIDTH<<"' before '"<<CNS_TAG_GRID<<"' given."<<std::endl;
                return false;
            }

            element = node->FirstChildElement(CNS_TAG_ROW);

            int i=0;
            while (i < height)
            {
                if (!element)
                {
                    std::cout << "Not enough '"<<CNS_TAG_ROW<<"' in '"<<CNS_TAG_GRID<<"' given."<<std::endl;
                    return false;
                }

                grid = element->GetText();
                int k = 0;
                text = "";
                int j = 0;

                for(k = 0; k < (strlen(grid)); k++)
                {
                    if (grid[k] == ' ')
                    {
                        stream<<text;
                        stream>>Grid[i][j];
                        stream.clear();
                        stream.str("");
                        text = "";
                        j++;
                    }
                    else
                    {
                        text += grid[k];
                    }
                }
                stream<<text;
                stream>>Grid[i][j];
                stream.clear();
                stream.str("");

                if (j < width-1)
                {
                    std::cout << "Not enough cells in '"<<CNS_TAG_ROW<<"' "<< i <<" given."<<std::endl;
                    return false;
                }

                i++;
                element = element->NextSiblingElement();
            }
        }

        node = map->IterateChildren(node);
    }

    return true;
}
