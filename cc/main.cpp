#include "tools.h"

#include <fstream>
#include <iostream>

#include "Config.h"
#include "Monos.h"

int main(int argc, char *argv[]) {
	setupEasylogging(argc, argv);

	Config config(argc, argv);

	if(config.isValid()) {
		Monos monos(config);
		monos.run();
	}

	return 0;
}
