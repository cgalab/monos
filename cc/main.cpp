#include "tools.h"

#include <fstream>
#include <iostream>

#include "Config.h"
#include "Monos.h"

int main(int argc, char *argv[]) {
	setupEasylogging(argc, argv);

	Args argPair({argc,argv});

	/* start */
	Monos monos(argPair);

	monos.run();

	return 0;
}
