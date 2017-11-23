#include "config.h"

Config::Config() : N(-1), searchParams(nullptr) {}

Config::Config(int numParams, float *paramArray) {
    N = numParams;
    searchParams = new float[N];

    for(int i = 0; i < N; ++i) {
       searchParams[i] = paramArray[i];
    }
}

Config::~Config() {
    if (searchParams) {
        delete[] searchParams;
    }
}

float Config::getParamValue(int i) const {
    return searchParams[i];
}

bool Config::getConfig(const char* FileName) {
    std::string value;
    float angle;
    int distance;
    float weight;
    int breakingties;
    unsigned int steplimit;
    float loglevel;
    float curvatureHeuriscitWeight;
    double pivotRadius = 0;
    float decreaseDistance;
    int distanceMin;
    int numOfParentsToIncreaseRadius;
    bool postsmoother;
    std::stringstream stream;

    TiXmlDocument doc(FileName);
    if (!doc.LoadFile()) {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    TiXmlElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "No '" << CNS_TAG_ROOT << "' element found in XML file." << std::endl;
        return false;
    }

    TiXmlElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if (!algorithm) {
        std::cout << "No '" << CNS_TAG_ALGORITHM << "' element found in XML file." << std::endl;
        return false;
    }

    TiXmlElement *element;

        N = CN_PT_NUM;
        searchParams = new float[N];

        element = algorithm->FirstChildElement(CNS_TAG_ANGLELIMIT);
        if (!element) {
            std::cout << "Error! No '"<< CNS_TAG_ANGLELIMIT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section." << std::endl;
            return false;
        } else {
            value = element->GetText();
            stream << value;
            stream >> angle;
            stream.clear();
            stream.str("");
            if (angle < 0) angle *= -1;
            if (angle > 180) {
                std::cout << "Warning! Trying to set angle limit to more than 180. " <<
                             "Angle limit is set to 180 instead." << std::endl;
                angle = 180;
            }
        }
        if (angle == 0) {
            std::cout << "Warning! '"<< CNS_TAG_ANGLELIMIT << "' is zero. Set to default value: " << CN_PTD_AL << "." << std::endl;
            angle = CN_PTD_AL;
        }
        searchParams[CN_PT_AL] = angle;


        element = algorithm->FirstChildElement(CNS_TAG_DISTANCE);
        if (!element) {
            std::cout << "Error! No '" << CNS_TAG_DISTANCE << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section." << std::endl;
            return false;
        } else {
            value = element->GetText();
            stream << value;
            stream >> distance;
            stream.clear();
            stream.str("");
            if (distance < 0) distance *= -1;
        }

        if (distance == 0) {
            std::cout << "Warning! Wrong '" << CNS_TAG_DISTANCE << "' element. Set to default value: " << CN_PTD_D
                      << "." << std::endl;
            distance = CN_PTD_D;
        }
        searchParams[CN_PT_D] = distance;


        element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_WEIGHT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_W << "." << std::endl;
            weight = CN_PTD_W;
        } else {
            value = element->GetText();
            stream << value;
            stream >> weight;
            stream.clear();
            stream.str("");
        }

        if (weight == 0) {
            std::cout << "Warning! Wrong '" << CNS_TAG_WEIGHT << "' element. Set to default value: " << CN_PTD_W
                      << "." << std::endl;
            weight = CN_PTD_W;
        }
        searchParams[CN_PT_W] = weight;

        element = algorithm->FirstChildElement(CNS_TAG_BREAKINGTIE);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_BREAKINGTIE << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CNS_TAG_ATTR_GMAX <<"." << std::endl;
            breakingties = CN_BT_GMAX;
        } else {
            value = element->GetText();
            if (value == CNS_TAG_ATTR_GMAX) {
                breakingties = CN_BT_GMAX;
            } else if (value == CNS_TAG_ATTR_GMIN) {
                breakingties = CN_BT_GMIN;
            } else {
                std::cout << "Warning! Wrong '" << CNS_TAG_BREAKINGTIE << "' element. Set to default value: \""
                          << CNS_TAG_ATTR_GMAX << "\"." << std::endl;
                breakingties = CN_BT_GMAX;
            }
        }
        searchParams[CN_PT_BT] = breakingties;


        element = algorithm->FirstChildElement(CNS_TAG_STEPLIMIT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_STEPLIMIT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            steplimit = 0;
        } else {
            value = element->GetText();
            if (value[0] == '-')  steplimit = 0;
            else {
                stream << value;
                stream >> steplimit;
                stream.clear();
                stream.str("");
            }
        }
        searchParams[CN_PT_SL] = steplimit;

        element = algorithm->FirstChildElement(CNS_TAG_CURVHEURWEIGHT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_CURVHEURWEIGHT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            curvatureHeuriscitWeight = 0;
        } else {
            value = element->GetText();
            stream << value;
            stream >> curvatureHeuriscitWeight;
            stream.clear();
            stream.str("");
        }
        searchParams[CN_PT_CHW] = curvatureHeuriscitWeight;

        element = algorithm->FirstChildElement(CNS_TAG_SMOOTHER);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_SMOOTHER << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            postsmoother = 0;
        } else {
            value = element->GetText();
            stream << value;
            stream >> postsmoother;
            stream.clear();
            stream.str("");
        }
        searchParams[CN_PT_PS] = postsmoother;

        element = algorithm->FirstChildElement(CNS_TAG_DECRDISTFACTOR);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_DECRDISTFACTOR << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_DDF << "." << std::endl;
            decreaseDistance = CN_PTD_DDF;
        } else {
            value = element->GetText();
            stream << value;
            stream >> decreaseDistance;
            stream.clear();
            stream.str("");
            if(decreaseDistance == 0) {
                std::cout << "Warning! Wrong '" << CNS_TAG_DECRDISTFACTOR << "Set to default value: " << CN_PTD_DDF
                          << "." << std::endl;
                decreaseDistance = CN_PTD_DDF;
            }
        }
        searchParams[CN_PT_DDF] = decreaseDistance;


        element = algorithm->FirstChildElement(CNS_TAG_DISTANCEMIN);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_DISTANCEMIN << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_DMIN << "." << std::endl;
            distanceMin = CN_PTD_DMIN;
        } else {
            value = element->GetText();
            stream << value;
            stream >> distanceMin;
            stream.clear();
            stream.str("");
            if(distanceMin == 0) {
                std::cout << "Warning! Wrong '" << CNS_TAG_DISTANCEMIN << "Set to default value: " << CN_PTD_DMIN
                          << "." << std::endl;
                distanceMin = CN_PTD_DMIN;
            }
        }
        searchParams[CN_PT_DM] = distanceMin;


        element = algorithm->FirstChildElement(CNS_TAG_PIVOTCIRCLE);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_PIVOTCIRCLE << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            pivotRadius = 0;
        } else {
            value = element->GetText();
            stream << value;
            stream >> pivotRadius;
            stream.clear();
            stream.str("");
            if (pivotRadius < 0) {
                pivotRadius *= -1;
            }
        }
        searchParams[CN_PT_PC] = pivotRadius;


        element = algorithm->FirstChildElement(CNS_TAG_NOFPTOINCRAD);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_NOFPTOINCRAD << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_NOFPTOINCRAD << "." << std::endl;
            numOfParentsToIncreaseRadius = CN_PTD_NOFPTOINCRAD;
        } else {
            value = element->GetText();
            stream << value;
            stream >> numOfParentsToIncreaseRadius;
            stream.clear();
            stream.str("");
            if(numOfParentsToIncreaseRadius==0) {
                std::cout << "Warning! Wrong '" << CNS_TAG_NOFPTOINCRAD << "Set to default value: " << CN_PTD_NOFPTOINCRAD
                          << "." << std::endl;
               numOfParentsToIncreaseRadius = CN_PTD_NOFPTOINCRAD;
            }
        }
        searchParams[CN_PT_NOP] = numOfParentsToIncreaseRadius;

    TiXmlElement *options = root->FirstChildElement(CNS_TAG_OPTIONS);
    if(!options) {
        std::cout << "Error! No '" << CNS_TAG_OPTIONS << "' element found in XML file." << std::endl;
        return false;
    }

    element = options->FirstChildElement(CNS_TAG_LOGLVL);
    if(!element) {
        std::cout << "Error! No '"<< CNS_TAG_LOGLVL << "' element found in XML file." << std::endl;
        return false;
    }

    value = element->GetText();
    stream << value;
    stream >> loglevel;
    stream.clear();
    stream.str("");

    searchParams[CN_PT_LOGLVL] = loglevel;

    return true;
}
