#include <stdexcept>

#include "logger.h"


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

	auto element = doc_.InsertEndChild(TiXmlElement(Logger::Tags::log));

	element->InsertEndChild(TiXmlElement(Logger::Tags::summary));

	if (logLevel_ > Logger::Levels::tiny) {
		element->InsertEndChild(TiXmlElement(Logger::Tags::path));

		element->InsertEndChild(TiXmlElement(Logger::Tags::angles));

		element->InsertEndChild(TiXmlElement(Logger::Tags::lpLevel));

		element->InsertEndChild(TiXmlElement(Logger::Tags::hpLevel));
	}

	if (logLevel_ >= Logger::Levels::med) {
		element->InsertEndChild(TiXmlElement(Logger::Tags::lowLevel));
	}

	if (logLevel_ >= Logger::Levels::iter) {
		element->InsertEndChild(TiXmlElement(Logger::Tags::iterations));
	}
}

Logger::~Logger() {
	doc_.SaveFile(logFileName_.c_str());
}
