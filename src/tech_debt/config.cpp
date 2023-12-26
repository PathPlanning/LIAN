#include "config.h"

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"
#include "xmlUtils.h"


const SearchParams& Config::params() const {
    return params_;
}

// const std::string& Config::getMapFileName() const {
//     return mapFileName_;
// }

Config::Config(const char* fileName) {
    TiXmlDocument doc(fileName);
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file.");
    }

    auto root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        throw std::runtime_error((std::stringstream() << "No '" << CNS_TAG_ROOT << "' element found in XML file.").str());
    }

    auto algorithm = getElement(root, CNS_TAG_ALGORITHM, CNS_TAG_ROOT);

    TiXmlElement* element;

    // element = getElement(root, CNS_TAG_MAP);
    // mapFileName_ = serialize<std::string>(element);

    element = getElement(algorithm, CNS_TAG_ANGLELIMIT, CNS_TAG_ALGORITHM);
    params_.angleLimit = std::min(std::abs(serialize<float>(element)), 180.f);

    element = getElement(algorithm, CNS_TAG_DISTANCE, CNS_TAG_ALGORITHM);
    params_.distance = std::abs(serialize<int>(element));

    // element = algorithm->FirstChildElement("isELian");
    // params_.doELian = serializeOrElse<int>(element, 0);

    element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
    params_.weight = serializeOrElse<float>(element, CN_PTD_W);

    element = algorithm->FirstChildElement(CNS_TAG_STEPLIMIT);
    params_.steplimit = serializeOrElse<unsigned int>(element, 0);

    element = algorithm->FirstChildElement(CNS_TAG_CURVHEURWEIGHT);
    params_.curvatureHeuristicWeight = serializeOrElse<float>(element, 0.0);

    element = algorithm->FirstChildElement(CNS_TAG_SMOOTHER);
    params_.postsmoother = serializeOrElse<bool>(element, false);

    element = algorithm->FirstChildElement(CNS_TAG_DECRDISTFACTOR);
    params_.decreaseDistanceFactor = serializeOrElse<float>(element, CN_PTD_DDF);

    element = algorithm->FirstChildElement(CNS_TAG_DISTANCEMIN);
    params_.distanceMin = serializeOrElse<int>(element, params_.distance * 0.1);

    element = algorithm->FirstChildElement(CNS_TAG_PIVOTCIRCLE);
    params_.pivotRadius = std::abs(serializeOrElse<double>(element, 0.0));

    element = algorithm->FirstChildElement(CNS_TAG_NOFPTOINCRAD);
    params_.numOfParentsToIncreaseRadius = serializeOrElse<int>(element, CN_PTD_NOFPTOINCRAD);

    TiXmlElement* options = getElement(root, CNS_TAG_OPTIONS);

    element = getElement(options, CNS_TAG_LOGLVL, CNS_TAG_OPTIONS);
    params_.logLevel = serialize<float>(element);
}
