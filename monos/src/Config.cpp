#include "Config.h"

bool Config::evaluateArguments(std::list<std::string> args) {
	std::string argument;
	bool ret = false;

	if(args.empty()) {
		printHelp();
		validConfig = false;
		return false;
	} else {
		while(!args.empty()) {
			argument = args.front();
			args.pop_front();
		    if (argument == "-help" || argument == "-h") {
				printHelp();
				printHelpLong();
				validConfig = false;
				return false;
			} else if (argument == "-v") {
				if(silent) {
					std::cout << "Use either verbose or silent, -v or -s, not both!";
				}
				verbose 		= true;
			} else if (argument == "-s") {
				if(verbose) {
					std::cout << "Use either verbose or silent, -v or -s, not both!";
				}
				silent 		= true;
			} else if (argument == "-obj") {
				if(args.empty()) {return false;}
				outputType = OutputType::OBJ;
				argument = args.front();
				args.pop_front();
				outputFileName = argument;
			} else if (argument == "-poly") {
				std::cout << ".poly is not supported yet!";
				validConfig = false;
				return false;
			} else if(args.empty()) {
				if(!fileExists(argument)) {
					std::cout << argument << " is no valid option or filename!";
					validConfig = false;
					return false;
				} else {
					fileName = argument;
					ret = true;
				}
			}
		}
	}

	return ret;
}

