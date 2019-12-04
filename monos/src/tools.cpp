/* monos is written in C++.  It computes the weighted straight skeleton
 * of a monotone polygon in asymptotic n log n time and linear space.
 *
 * Copyright 2018, 2019 Günther Eder - geder@cs.sbg.ac.at
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../easyloggingpp/src/easylogging++.h"
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
