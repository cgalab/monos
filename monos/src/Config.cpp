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

#include "Config.h"

#include <stdlib.h>

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
			not_x_mon = true;
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

