#include "cConfig.h"
#include <algorithm>
#include <sstream>
cConfig::cConfig()
{
    N = -1;
    searchParams = 0;
}

cConfig::cConfig(int numParams, float *paramArray)
{
    N = numParams;
    searchParams = new float[N];

    for(int i = 0; i < N; i++)
    {
       searchParams[i] = paramArray[i];
    }
}

cConfig::~cConfig()
{
    if (searchParams)
    {
        delete[] searchParams;
    }
}

bool cConfig::getConfig(const char* FileName)
{
    std::string value;
    float angle;
    int distance;
    float weight;
    unsigned int steplimit;
    float loglevel;
    float circleRadiusFactor;
    float curvatureHeuriscitWeight;
    float linecost;
    bool checkLesserCircle=false;
    float decreaseDistance;
    int distanceMin;
    int numOfParentsToIncreaseRadius;
    std::stringstream stream;

    TiXmlDocument doc(FileName);
    if(!doc.LoadFile())
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    TiXmlElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return false;
    }

    TiXmlElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if (!algorithm)
    {
        std::cout << "No 'algorithm' element found in XML file."<<std::endl;
        return false;
    }

    TiXmlElement *element;

        N = CN_PT_NUM;
        searchParams = new float[N];

        element = algorithm->FirstChildElement(CNS_TAG_ANGLELIMIT);
        if (!element)
        {
            std::cout << "Error! No '"<<CNS_TAG_ANGLELIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section." << std::endl;
            return false;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>angle;
            stream.clear();
            stream.str("");
            if (angle < 0)
            {
                angle *= -1;
            }
            if (angle > 180)
            {
                std::cout << "Warning! Trying to set angle limit to more than 180. " <<
                             "Angle limit is set to 180 instead." << std::endl;
                angle = 180;
            }
        }

        if (angle == 0)
        {
            std::cout << "Warning! '"<<CNS_TAG_ANGLELIMIT<<"' is zero. It's compared to " << CN_PTD_AL <<"."<< std::endl;
            angle = CN_PTD_AL;
        }
        searchParams[CN_PT_AL] = angle;


        element = algorithm->FirstChildElement(CNS_TAG_DISTANCE);
        if (!element)
        {
            std::cout << "Error! No '"<<CNS_TAG_DISTANCE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section." << std::endl;
            return false;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>distance;
            stream.clear();
            stream.str("");
            if (distance < 0)
            {
                distance *= -1;
            }
        }

        if (distance == 0)
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_DISTANCE<<"' section. It's compared to " << CN_PTD_D <<"."<<std::endl;
            distance = CN_PTD_D;
        }
        searchParams[CN_PT_D] = distance;


        element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
        if (!element)
        {
            std::cout << "Error! No '"<<CNS_TAG_WEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<<CN_PTD_W<<"."<<std::endl;
            weight = CN_PTD_W;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>weight;
            stream.clear();
            stream.str("");
        }

        if (weight == 0)
        {
            std::cout << "Warning! Wrong '"<<CNS_TAG_WEIGHT<<"' section. It's compared to " << CN_PTD_W <<"."<<std::endl;
            weight = CN_PTD_W;
        }
        searchParams[CN_PT_W] = weight;


        element = algorithm->FirstChildElement(CNS_TAG_STEPLIMIT);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_STEPLIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 0."<< std::endl;
            steplimit = 0;
        }
        else
        {
            value = element->GetText();
            if (value[0] == '-')
            {
                steplimit = 0;
            }
            else
            {
                stream<<value;
                stream>>steplimit;
                stream.clear();
                stream.str("");
            }
        }
        searchParams[CN_PT_SL] = steplimit;


        element = algorithm->FirstChildElement(CNS_TAG_CRADIUSFACTOR);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_CRADIUSFACTOR<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 0."<< std::endl;
            circleRadiusFactor = 0;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>circleRadiusFactor;
            stream.clear();
            stream.str("");
        }
        searchParams[CN_PT_CRF] = circleRadiusFactor;


        element = algorithm->FirstChildElement(CNS_TAG_CURVHEURWEIGHT);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_CURVHEURWEIGHT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to 0."<<std::endl;
            curvatureHeuriscitWeight = 0;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>curvatureHeuriscitWeight;
            stream.clear();
            stream.str("");
        }
        searchParams[CN_PT_CHW] = curvatureHeuriscitWeight;


        element = algorithm->FirstChildElement(CNS_TAG_DECRDISTFACTOR);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_DECRDISTFACTOR<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<<CN_PTD_DDF<<"."<<std::endl;
            decreaseDistance = CN_PTD_DDF;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>decreaseDistance;
            stream.clear();
            stream.str("");
            if(decreaseDistance==0)
            {
                std::cout << "Wrong '"<<CNS_TAG_DECRDISTFACTOR<<"' element."<<std::endl;
                decreaseDistance = CN_PTD_DDF;
            }
        }
        searchParams[CN_PT_DDF] = decreaseDistance;


        element = algorithm->FirstChildElement(CNS_TAG_DISTANCEMIN);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_DISTANCEMIN<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<<CN_PTD_DMIN<<"."<<std::endl;
            distanceMin = CN_PTD_DMIN;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>distanceMin;
            stream.clear();
            stream.str("");
            if(distanceMin==0)
            {
                std::cout << "Wrong '"<<CNS_TAG_DISTANCEMIN<<"' element."<<std::endl;
                distanceMin = CN_PTD_DMIN;
            }
        }
        searchParams[CN_PT_DM] = distanceMin;


        element = algorithm->FirstChildElement(CNS_TAG_LINECOST);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_LINECOST<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<<CN_PTD_LC<<"."<<std::endl;
            linecost = CN_PTD_LC;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>linecost;
            stream.clear();
            stream.str("");
            if(linecost==0)
            {
                std::cout << "Wrong '"<<CNS_TAG_LINECOST<<"' element."<<std::endl;
                linecost = CN_PTD_LC;
            }
        }
        searchParams[CN_PT_LC] = linecost;


        element = algorithm->FirstChildElement(CNS_TAG_CHECKCIRCLE);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_CHECKCIRCLE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<<CNS_TAG_ATTR_FALSE<<"."<<std::endl;
            checkLesserCircle = 0;
        }
        else
        {
            value = element->GetText();
            if (value == CNS_TAG_ATTR_FALSE)
            {
               checkLesserCircle = 0;
            }
            else if (value == CNS_TAG_ATTR_TRUE)
            {
                checkLesserCircle = 1;
            }
            else
            {
               std::cout << "Wrong '"<<CNS_TAG_CHECKCIRCLE<<"' element."<<std::endl;
               checkLesserCircle = 0;
            }
        }
        searchParams[CN_PT_CLC] = checkLesserCircle;


        element = algorithm->FirstChildElement(CNS_TAG_NOFPTOINCRAD);
        if (!element)
        {
            std::cout << "No '"<<CNS_TAG_NOFPTOINCRAD<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to "<<CN_PTD_NOFPTOINCRAD<<"."<<std::endl;
            numOfParentsToIncreaseRadius = CN_PTD_NOFPTOINCRAD;
        }
        else
        {
            value = element->GetText();
            stream<<value;
            stream>>numOfParentsToIncreaseRadius;
            stream.clear();
            stream.str("");
            if(numOfParentsToIncreaseRadius==0)
            {
               std::cout << "Wrong '"<<CNS_TAG_NOFPTOINCRAD<<"' element."<<std::endl;
               numOfParentsToIncreaseRadius = CN_PTD_NOFPTOINCRAD;
            }
        }
        searchParams[CN_PT_NOP] = numOfParentsToIncreaseRadius;

    TiXmlElement *options = root->FirstChildElement(CNS_TAG_OPTIONS);
    if(!options)
    {
        std::cout << "No '"<<CNS_TAG_OPTIONS<<"' element found in XML file."<<std::endl;
        return false;
    }

    element = options->FirstChildElement(CNS_TAG_LOGLVL);
    if(!element)
    {
        std::cout << "No '"<<CNS_TAG_LOGLVL<<"' element found in XML file."<<std::endl;
        return false;
    }

    value = element->GetText();
    stream<<value;
    stream>>loglevel;
    stream.clear();
    stream.str("");

    searchParams[CN_PT_LOGLVL] = loglevel;

    return true;
}
