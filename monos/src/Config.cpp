#include "Config.h"


bool Config::evaluateArguments(int argc, char *argv[]) {
	while (1) {
		int option_index = 0;
		int r = getopt_long(argc, argv, "hO:R:", long_options, &option_index);

		if (r == -1) break;
		switch (r) {
		case 'h':
			usage(argv[0], 0);
			break;

		case 'v':
			verbose = true;
			silent  = false;
			resetLogging(true);
			break;

		case 'n':
			normalize = true;
			break;

		case 'o':
			outputFileName = std::string(optarg);
			break;

		case 't':
			timings = true;
			break;

		case 'x':
			x_mon = true;
			break;

		default:
			std::cerr << "Invalid option " << (char)r << std::endl;
			validConfig = false;
			exit(1);
		}
	}

	if (argc - optind > 1) {
		usage(argv[0], 1);
	}

	use_stdin = true;
	if (argc - optind == 1) {
		std::string fn(argv[optind]);
		if (fn != "-") {
			fileName  = fn;
			use_stdin = false;
		}
	}

	return true;
}

