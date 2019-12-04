/*
 * monos is written in C++.  It computes the weighted straight skeleton
 * of a monotone polygon in asymptotic n log n time and linear space.
 * Copyright (C) 2018 - Günther Eder - geder@cs.sbg.ac.at
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

static struct option long_options[] = {
		{ "help"        , no_argument      , 0, 'h'},
		{ "verbose"     , no_argument      , 0, 'v'},
		{ "xmon"        , no_argument      , 0, 'x'},
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

		fprintf(f,"Usage: %s [options] <GRAPHML file>\n", progname);
		fprintf(f,"  Options: --out \t| --o <filename> \t write output\n");
		fprintf(f,"           --verbose \t| --v \t\t\t print processing information\n");
		fprintf(f,"           --mon \t| --x \t\t\t monotone but not x-monotone (works by default in master branch)\n");
		fprintf(f,"           --timings \t| --t \t\t\t print timings [ms]\n");
		fprintf(f,"           --normalize \t| --n \t\t\t write output normalized to the origin\n");
		fprintf(f,"\n");
		fprintf(f,"Input format is .gml/.graphml (GraphML).\n");
		fprintf(f,"Parsing input from cin assumes graphml format.\n");
		fprintf(f,"\n");
		exit(err);
	}

	Config(bool _gui = false):
		fileName(""),gui(_gui),
		outputFileName(""),validConfig(false) {
	}

	Config(int argc, char *argv[], bool gui = false):Config(gui) {
		validConfig = evaluateArguments(argc,argv);
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
	bool			not_x_mon = false;

	bool			duplicate = false;
	int				copies	  = 2;

	bool 			gui;

	std::string		outputFileName;

private:
	bool evaluateArguments(int argc, char *argv[]);

	std::string 	printOptions;
	bool	 		validConfig;
};

#endif /* CONFIG_H_ */
