#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
#include <list>
#include <fstream>

#include <ctime>
#include <time.h>

#include <getopt.h>

#include "Definitions.h"
#include "tools.h"

using Args = std::pair<int,char**>;

static struct option long_options[] = {
		{ "help"        , no_argument      , 0, 'h'},
		{ "verbose"     , no_argument      , 0, 'v'},
		{ "cgal"   		, no_argument      , 0, 'c'},
		{ "normalize"   , no_argument      , 0, 'n'},
		{ "timings"     , no_argument      , 0, 't'},
		{ "out"         , required_argument, 0, 'o'},
		{ 0, 0, 0, 0}
};

class Config {
public:

	[[noreturn]]
	 static void
	 usage(const char *progname, int err) {
		FILE *f = err ? stderr : stdout;

		fprintf(f,"Usage: %s [options] <OBJ|GRAPHML file>\n", progname);
		fprintf(f,"  Options: --out \t| -o <filename> \t write output\n");
		fprintf(f,"           --verbose \t| -v \t\t\t print processing information\n");
		fprintf(f,"           --cgal \t| -c \t\t\t use cgal implementation (for timing and testing)\n");
		fprintf(f,"           --timings \t| -t \t\t\t print timings [ms]\n");
		fprintf(f,"           --normalize \t| -n \t\t\t write output normalized to the origin\n");
		fprintf(f,"\n");
		fprintf(f,"Input format can be .gml/.graphml (GraphML) or .obj (Wavefront Object).\n");
		fprintf(f,"Parsing input from cin assumes graphml format.");
		fprintf(f,"\n");
		exit(err);
	}

	Config(bool _gui = false):
		fileName(""),gui(_gui),
		outputType(OutputType::NONE),outputFileName(""),validConfig(false) {
	}

	Config(Args args, bool gui = false):Config(gui) {
		validConfig = evaluateArguments(args);
	}

	bool isValid() const { return validConfig; }

	void setNewInputfile(const std::string& _fileName) {
		if(fileExists(_fileName)) {
			fileName = _fileName;
			validConfig = true;
		}
	}

	std::string getFileNameNoPath() const {
		return fileName.substr(fileName.find_last_of("/\\") + 1);
	}

	std::string   	fileName;

	bool 			use_stdin = false;

	bool 			verbose   = false;
	bool			silent    = true;
	bool 			normalize = false;
	bool 			timings   = false;
	bool 			gui;

	bool			run_cgal_code = false;

	OutputType  	outputType;
	std::string		outputFileName;

private:
	bool evaluateArguments(Args args);

	std::string 	printOptions;
	bool	 		validConfig;
};

#endif /* CONFIG_H_ */
