#pragma once

#include <string>
#include <exception>
#include <optional>
#include <vector>
#include <sstream>
#include "tinyxml/tinyxml.h"

template <typename T>
T serialize(TiXmlElement* element) {
    std::stringstream ss;
    ss << std::string(element->GetText());
    T res;
    ss >> res;
    return res;
}

template <>
bool serialize<bool>(TiXmlElement* element) {
    std::string buf(element->GetText());
    if (buf == "true") {
        return true;
    }
    if (buf == "false") {
        return false;
    }
    throw std::runtime_error("Expected true or false. Got: " + buf);
}

template <typename T>
T serializeOrElse(TiXmlElement* element, const T& orElse) {
    if (!element) {
        return orElse;
    }
    return serialize<T>(element);
}

template <typename T>
std::optional<T> serializeOpt(TiXmlElement* element) {
    if (!element) {
        return std::nullopt;
    }
    return serialize<T>(element);
}

template <typename T>
std::vector<T> serializeVector(TiXmlElement* element) {
    std::stringstream ss;
    ss << std::string(element->GetText());
    T cell;
    std::vector<T> res;
    while (ss >> cell) {
        res.push_back(cell);
    }

    return res;
}

TiXmlElement* getElement(
    TiXmlElement* section, const std::string& elementName,
    std::optional<std::string> sectionName = std::nullopt) {
    TiXmlElement* element = section->FirstChildElement(elementName.c_str());
    if (!element) {
        std::stringstream errorStream;
        errorStream << "Error! No '" << elementName << "' element found";
        if (sectionName) {
            errorStream << " inside '" << (*sectionName) << "' section.";
        }
        throw std::runtime_error(errorStream.str());
    }

    return element;
}
