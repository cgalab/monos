#include "tools.h"

#include <fstream>
#include <iostream>

#include "Config.h"
#include "Monos.h"

int main(int argc, char *argv[]) {
	setupEasylogging(argc, argv);

//	static std::list<std::string> args;
//	for(auto i = 1; i < argc; ++i) { args.push_back(std::string(argv[i])); }
	Args argPair({argc,argv});

	/* start */
	Monos monos(argPair);

	monos.run();

	return 0;
}
