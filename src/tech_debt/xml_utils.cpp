#include "xml_utils.h"

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

TiXmlElement* getElement(
    TiXmlElement* section, const std::string& elementName,
    std::optional<std::string> sectionName) {
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
