#include <stdexcept>

#include "logger.h"

Logger::Logger(float loglvl) : logLevel_(loglvl) {
}

Logger::~Logger() {
    if (doc_) {
        doc_->Clear();
        delete doc_;
    }
}

bool Logger::getLog(const std::string& fileName) {
    std::string value;
    TiXmlDocument doc_xml(fileName.c_str());

    if (!doc_xml.LoadFile()) {
        throw std::runtime_error("Error opening XML-file in getLog");
        return false;
    }

    value = fileName;
    size_t dotPos = value.find_last_of(".");

    if (dotPos != std::string::npos) {
        value.insert(dotPos, CN_LOG);
    }
    else {
        value += CN_LOG;
    }

    logFileName_ = value;
    doc_xml.SaveFile(logFileName_.c_str());

    doc_ = new TiXmlDocument(logFileName_.c_str());
    doc_->LoadFile();

    TiXmlElement* msg;
    TiXmlElement* root;

    root = doc_->FirstChildElement(CNS_TAG_ROOT);
    TiXmlElement* log = new TiXmlElement(CNS_TAG_LOG);
    root->LinkEndChild(log);

    msg = new TiXmlElement(CNS_TAG_MAPFN);
    msg->LinkEndChild(new TiXmlText(fileName.c_str()));
    log->LinkEndChild(msg);

    msg = new TiXmlElement(CNS_TAG_SUM);
    log->LinkEndChild(msg);

    if (logLevel_ > CN_LOGLVL_TINY) {

        TiXmlElement* path = new TiXmlElement(CNS_TAG_PATH);
        log->LinkEndChild(path);

        TiXmlElement* angles = new TiXmlElement(CNS_TAG_ANGLES);
        log->LinkEndChild(angles);

        TiXmlElement* lplevel = new TiXmlElement(CNS_TAG_LPLEVEL);
        log->LinkEndChild(lplevel);

        TiXmlElement* hplevel = new TiXmlElement(CNS_TAG_HPLEVEL);
        log->LinkEndChild(hplevel);
    }

    if (logLevel_ >= CN_LOGLVL_MED) {
        TiXmlElement* lowlevel = new TiXmlElement(CNS_TAG_LOWLEVEL);
        log->LinkEndChild(lowlevel);
    }

    if (logLevel_ - CN_LOGLVL_ITER < 0.001) {
        TiXmlElement* iters = new TiXmlElement(CNS_TAG_ITERS);
        log->LinkEndChild(iters);
    }

    return true;
}

void Logger::saveLog() {
    doc_->SaveFile(logFileName_.c_str());
}
