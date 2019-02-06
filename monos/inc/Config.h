#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
#include <list>

#include "Definitions.h"
#include "tools.h"

class Config {
public:
	Config(bool _gui = false):
		fileName(""),verbose(false),silent(false),gui(_gui),
		outputType(OutputType::NONE),outputFileName(""),validConfig(false) {

		printOptions = "[-h] [-v|-s] ";
		printOptions += "[-poly|-obj <filename>] <filename>";
	}

	Config(std::list<std::string> args, bool gui = false):Config(gui) {
		validConfig = evaluateArguments(args);
	}

	void printHelp() const {
		std::cout << "usage: monos " << printOptions << std::endl;
	}

	void printHelpLong() const {
		std::cout << "  -h \t\t\tpruint this help" << std::endl
			 << "  -v \t\t\tverbose mode, shows more information about the computation" << std::endl
			 << "  -s \t\t\tsilent mode, shows no information" << std::endl
			 << "  -l \t\t\tlogging verbose output to <filename>.log" << std::endl
			 << "  -obj <file> \t\twrite output in wavefront obj format (3D coordinates)" << std::endl
			 << "  <filename> \t\tinput type can be .gml (GraphML) or .obj (Wavefront Object)" << std::endl;
	}

	bool isValid() const { return validConfig; }

	void setNewInputfile(const std::string& _fileName) {
		if(fileExists(_fileName)) {
			fileName = _fileName;
			validConfig = true;
		}
	}

	std::string   	fileName;

	bool 			verbose;
	bool			silent;
	bool 			gui;

	OutputType  	outputType;
	std::string		outputFileName;

private:
	bool evaluateArguments(std::list<std::string> args);

	std::string 	printOptions;
	bool	 		validConfig;
};

#endif /* CONFIG_H_ */
