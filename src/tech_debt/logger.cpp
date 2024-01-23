#include <stdexcept>

#include "logger.h"

Logger::Tags Logger::tags = {};

Logger::Logger(int loglvl, const std::string &configFileName) : doc_(configFileName.c_str()), logLevel_(loglvl) {
	if (!doc_.LoadFile()) {
		throw std::runtime_error("Error opening XML-file in getLog");
	}

	std::string value = configFileName;
	size_t dotPos = value.find_last_of(".");

	if (dotPos != std::string::npos) {
		value.insert(dotPos, CN_LOG);
	}
	else {
		value += CN_LOG;
	}
	logFileName_ = value;

	auto root = doc_.FirstChild(Logger::tags.root);

	auto element = root->InsertEndChild(TiXmlElement(Logger::tags.log));

	element->InsertEndChild(TiXmlElement(Logger::tags.summary));

	if (logLevel_ > CN_LOGLVL_TINY) {
		element->InsertEndChild(TiXmlElement(Logger::tags.path));

		element->InsertEndChild(TiXmlElement(Logger::tags.angles));

		element->InsertEndChild(TiXmlElement(Logger::tags.lpLevel));

		element->InsertEndChild(TiXmlElement(Logger::tags.hpLevel));
	}

	if (logLevel_ >= CN_LOGLVL_MED) {
		element->InsertEndChild(TiXmlElement(Logger::tags.lowLevel));
	}

	if (logLevel_ >= CN_LOGLVL_ITER) {
		element->InsertEndChild(TiXmlElement(Logger::tags.iterations));
	}
}

Logger::~Logger() {
	doc_.SaveFile(logFileName_.c_str());
}
