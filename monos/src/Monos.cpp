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


#include <iostream>
#include <exception>

#include "Monos.h"
#include "Data.h"


Monos::Monos(std::list<std::string>& args):config(args),wf(data),s(data,wf) {
	/* evaluate arguments */
	if(!config.isValid()) {return;}

	/* load data: input and bbox */
	data.initialize(config);
	LOG(INFO) << "input loaded";

	if(config.verbose) {data.printInput();}
	LOG(INFO) << "print input done";

	/* initialize wavefront and skeleton */
//	s  = Skeleton(data);
	wf.InitializeEventsAndPathsPerEdge();
	wf.InitializeNodes();
}

Monos::~Monos() {}

bool Monos::init() {
	if(!config.isValid()) {return false;}

	/* verfify monotonicity -- if required, rotate to assure
	 * that P is x-monotone */
	if(!data.ensureMonotonicity()) {
		LOG(WARNING) << "polygon is not monotone!";
		return false;
	}

	/* compute BBox and min/max monotonicity vertices */
	data.bbox = data.computeBoundingBox();

	/** input must be x-monotone */
	wf.ChainDecomposition();
	LOG(INFO) << "chain decomposition done";

	return true;
}


void Monos::start() {
	if(!init()) {return;}

	if(!wf.ComputeSkeleton(true)) {return;}
	LOG(INFO) << "lower skeleton done";

	if(!wf.ComputeSkeleton(false)) {return;}
	LOG(INFO) << "upper skeleton done";

	/* sort nodes s.t. incident 'arcs' are in correct order, i.e.,
	 * their incidences. */
	wf.SortArcsOnNodes();

	s.MergeUpperLowerSkeleton();
	LOG(INFO) << "merging upper and lower skeleton done";

	if(config.outputType != OutputType::NONE) {
		s.writeOBJ(config);
		data.addPolyToOBJ(config);
		LOG(INFO) << "output written";
	}
}
