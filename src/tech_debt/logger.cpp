#include <stdexcept>

#include "logger.h"

Logger::Logger(int loglvl, const std::string& configFileName) : logLevel_(loglvl) {
    std::string value = configFileName;
    size_t dotPos = value.find_last_of(".");

    if (dotPos != std::string::npos) {
        value.insert(dotPos, CN_LOG);
    }
    else {
        value += CN_LOG;
    }
    logFileName_ = value;


    doc_.InsertEndChild(TiXmlDeclaration("1.0", "UTF-8", ""));

    auto element = doc_.InsertEndChild(TiXmlElement(CNS_TAG_LOG));

    element->InsertEndChild(TiXmlElement(CNS_TAG_SUM));

    if (logLevel_ > CN_LOGLVL_TINY) {
        element->InsertEndChild(TiXmlElement(CNS_TAG_PATH));

        element->InsertEndChild(TiXmlElement(CNS_TAG_ANGLES));

        element->InsertEndChild(TiXmlElement(CNS_TAG_LPLEVEL));

        element->InsertEndChild(TiXmlElement(CNS_TAG_HPLEVEL));
    }

    if (logLevel_ >= CN_LOGLVL_MED) {
        element->InsertEndChild(TiXmlElement(CNS_TAG_LOWLEVEL));
    }

    if (logLevel_ >= CN_LOGLVL_ITER) {
        element->InsertEndChild(TiXmlElement(CNS_TAG_ITERS));
    }
}

Logger::~Logger() {
    doc_.SaveFile(logFileName_.c_str());
}
