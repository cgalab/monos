
#include "tools.h"

INITIALIZE_EASYLOGGINGPP

bool fileExists(const std::string& fileName) {
  struct stat buffer;
  return (stat (fileName.c_str(), &buffer) == 0);
}

std::string currentTimeStamp() {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
	std::string str(buffer);
	return str;
}

void getNormalizer(const BBox& bbox, double& xt, double& xm, double& yt, double& ym, double& zt, double& zm) {
	double x_span  = (1.0/OBJSCALE) * (bbox.xMax.doubleValue() - bbox.xMin.doubleValue());
	double y_span  = (1.0/OBJSCALE) * (bbox.yMax.doubleValue() - bbox.yMin.doubleValue());

	xt = bbox.xMin.doubleValue() + (0.5 * (OBJSCALE) * x_span);
	yt = bbox.yMin.doubleValue() + (0.5 * (OBJSCALE) * y_span);
	zt = 0.0;

	xm = (x_span != 0) ? OBJSCALE/x_span : 1;
	ym = (y_span != 0) ? OBJSCALE/y_span : 1;

	xm = ym = std::max(xm,ym);
	zm = xm;
}


void setupEasylogging(int argc, char** argv) {
	START_EASYLOGGINGPP(argc, argv);
	//ELPP_NO_DEFAULT_LOG_FILE
	el::Configurations defaultConf;
	defaultConf.setToDefault();
	//defaultConf.setGlobally(el::ConfigurationType::Format, "%datetime{H:%m:%s.%g} %level %func: %msg");
	defaultConf.setGlobally(el::ConfigurationType::Format, "%msg");
	el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
	el::Loggers::setLoggingLevel(el::Level::Global);
	el::Loggers::addFlag(el::LoggingFlag::ColoredTerminalOutput);

	defaultConf.setGlobally(el::ConfigurationType::ToFile, "false");
	defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");

	el::Loggers::reconfigureAllLoggers(defaultConf);
	el::Loggers::reconfigureLogger("default", defaultConf);
}

void resetLogging(bool output) {
	el::Configurations defaultConf;
	defaultConf.setToDefault();
	defaultConf.setGlobally(el::ConfigurationType::Format, "%msg");
	el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
	el::Loggers::setLoggingLevel(el::Level::Global);
	el::Loggers::addFlag(el::LoggingFlag::ColoredTerminalOutput);

	defaultConf.setGlobally(el::ConfigurationType::ToFile, "false");

	if(output) {
		defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "true");
	} else {
		defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
	}
	el::Loggers::reconfigureAllLoggers(defaultConf);
}
