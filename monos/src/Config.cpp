#include "Config.h"


bool Config::evaluateArguments(Args args) {
	while (1) {
		int option_index = 0;
		int r = getopt_long(args.first, args.second, "hO:R:", long_options, &option_index);

		if (r == -1) break;
		switch (r) {
		case 'h':
			usage(args.second[0], 0);
			break;

		case 'v':
			verbose = true;
			silent  = false;
			resetLogging(true);
			break;

		case 'o':
			outputFileName = std::string(optarg);
			outputType = OutputType::OBJ;
			break;

		default:
			std::cerr << "Invalid option " << (char)r << std::endl;
			validConfig = false;
			exit(1);
		}
	}

	if (args.first - optind > 1) {
		usage(args.second[0], 1);
	}

	use_stdin = true;
	if (args.first - optind == 1) {
		std::string fn(args.second[optind]);
		if (fn != "-") {
			fileName  = fn;
			use_stdin = false;
		}
	}

	return true;
}

