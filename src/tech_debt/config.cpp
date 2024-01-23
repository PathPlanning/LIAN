#include "config.h"

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"
#include "xml_utils.h"

namespace {
    constexpr auto tagRoot = "root";
    constexpr auto tagAlgorithm = "algorithm";
    constexpr auto tagAngleLimit = "anglelimit";
    constexpr auto tagDistance = "distance";
    constexpr auto tagWeight = "hweight";
    constexpr auto tagStepLimit = "steplimit";
    constexpr auto tagCurvHeurWeight = "curvatureHeuristicWeight";
    constexpr auto tagSmoother = "postsmoothing";
    constexpr auto tagPivotCircle = "pivotCircleRadius";
    constexpr auto tagDistanceMin = "distancemin";
    constexpr auto tagDecriDistFactor = "decreaseDistanceFactor";
    constexpr auto tagNumOfParentsToIncRadius = "numOfParentsToIncreaseRadius";
    constexpr auto tagOptions = "options";
    constexpr auto tagLogLevel = "loglevel";
 
    constexpr auto defaultWeight = 2;

    constexpr auto CN_PTD_DDF = 1;// Value that divides distance in order to decrease radius of the step

    constexpr auto CN_PTD_NOFPTOINCRAD = 2;// Number of parent vertecies that must have the same radius in order to start the increase of current radius
}

const SearchParams& Config::params() const {
    return params_;
}

Config::Config(const char* fileName) {
    TiXmlDocument doc(fileName);
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file.");
    }

    auto root = doc.FirstChildElement(tagRoot);
    if (!root) {
        std::ostringstream err_oss;
        err_oss << "No '" << tagRoot << "' element found in XML file.";
        throw std::runtime_error(err_oss.str());
    }

    auto algorithm = getElement(root, tagAlgorithm, tagRoot);

    TiXmlElement* element;

    // element = getElement(root, CNS_TAG_MAP);
    // mapFileName_ = serialize<std::string>(element);

    element = getElement(algorithm, tagAngleLimit, tagAlgorithm);
    params_.angleLimit = std::min(std::abs(serialize<float>(element)), 180.f);

    element = getElement(algorithm, tagDistance, tagAlgorithm);
    params_.distance = std::abs(serialize<int>(element));

    // element = algorithm->FirstChildElement("isELian");
    // params_.doELian = serializeOrElse<int>(element, 0);

    element = algorithm->FirstChildElement(tagWeight);
    params_.weight = serializeOrElse<float>(element, defaultWeight);

    element = algorithm->FirstChildElement(tagStepLimit);
    params_.stepLimit = serializeOrElse<unsigned int>(element, 0);

    element = algorithm->FirstChildElement(tagCurvHeurWeight);
    params_.curvatureHeuristicWeight = serializeOrElse<float>(element, 0.0);

    element = algorithm->FirstChildElement(tagSmoother);
    params_.postsmoother = serializeOrElse<bool>(element, false);

    element = algorithm->FirstChildElement(tagDecriDistFactor);
    params_.decreaseDistanceFactor = serializeOrElse<float>(element, CN_PTD_DDF);

    element = algorithm->FirstChildElement(tagDistanceMin);
    params_.distanceMin = serializeOrElse<int>(element, params_.distance * 0.1);

    element = algorithm->FirstChildElement(tagPivotCircle);
    params_.pivotRadius = std::abs(serializeOrElse<double>(element, 0.0));

    element = algorithm->FirstChildElement(tagNumOfParentsToIncRadius);
    params_.numOfParentsToIncreaseRadius = serializeOrElse<int>(element, CN_PTD_NOFPTOINCRAD);

    TiXmlElement* options = getElement(root, tagOptions);

    element = getElement(options, tagLogLevel, tagOptions);
    params_.logLevel = serialize<int>(element);
}
