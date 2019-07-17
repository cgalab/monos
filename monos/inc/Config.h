#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
#include <list>
#include <fstream>

#include <getopt.h>

#include "Definitions.h"
#include "tools.h"

using Args = std::pair<int,char**>;

static struct option long_options[] = {
		{ "help"        , no_argument      , 0, 'h'},
		{ "verbose"     , no_argument      , 0, 'v'},
		{ "obj"         , required_argument, 0, 'o'},
		{ 0, 0, 0, 0}
};

class Config {
public:

	[[noreturn]]
	 static void
	 usage(const char *progname, int err) {
		FILE *f = err ? stderr : stdout;

		fprintf(f,"Usage: %s [options] <OBJ|GRAPHML file>\n", progname);
		fprintf(f,"  Options: --obj | -o <filename>      write output.\n");
		fprintf(f,"           --verbose | -v             print processing information.\n");
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

	std::string   	fileName;

	bool 			use_stdin = false;

	bool 			verbose = false;
	bool			silent  = true;
	bool 			gui;

	OutputType  	outputType;
	std::string		outputFileName;

private:
	bool evaluateArguments(Args args);

	std::string 	printOptions;
	bool	 		validConfig;
};

#endif /* CONFIG_H_ */
