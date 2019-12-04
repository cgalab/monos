/* monos is written in C++.  It computes the weighted straight skeleton
 * of a monotone polygon in asymptotic n log n time and linear space.
 *
 * Copyright 2018, 2019 Günther Eder - geder@cs.sbg.ac.at
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
