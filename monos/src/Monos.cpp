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


Monos::Monos(Args args, bool gui):
config(args,gui) {
	/* evaluate arguments */
	if(!config.isValid() && !gui) {return;}

	constructorInit(gui);
}

Monos::~Monos() {
	destruct();
}

void Monos::constructorInit(bool gui) {
	data = new Data(gui);

	/* load data: input and bbox */
	data->initialize(config);
	if(config.verbose) {
		LOG(INFO) << "input loaded";
	}

	if(config.verbose) {
		data->printInput();
		LOG(INFO) << "print input done";
	}

	wf = new Wavefront(*data);
	s  = new Skeleton(*data,*wf);

	/* initialize wavefront and skeleton */
	wf->InitializeEventsAndPathsPerEdge();
	wf->InitializeNodes();
}

void Monos::reinitialize(const std::string& fileName, bool gui) {
	if(fileExists(fileName)) {
		if(config.verbose) {
			LOG(INFO) << "File " << fileName << " exists, reloading!";
		}
		destruct();

		config.setNewInputfile(fileName);
		constructorInit(gui);
	}
}

void Monos::destruct() {
	delete data;
	delete wf;
	delete s;
}

void Monos::run() {
	clock_t begin, end;
	if(config.timings) {
		begin = clock();
	}

	if(!config.run_cgal_code) {

		/**************************************************************/
		/*				MONOTONE SKELETON APPROACH 					  */
		/**************************************************************/

		if(!init()) {return;}

		if(!wf->ComputeSkeleton(true)) {return;}
		if(config.verbose) {LOG(INFO) << "lower skeleton done";}

		if(!wf->ComputeSkeleton(false)) {return;}
		if(config.verbose) {LOG(INFO) << "upper skeleton done";}

		/* sort nodes s.t. incident 'arcs' are in correct order, i.e.,
		 * their incidences. */
		wf->SortArcsOnNodes();

		s->MergeUpperLowerSkeleton();
		if(config.verbose) {LOG(INFO) << "merging upper and lower skeleton done";}

	} else {
		/**************************************************************/
		/*				CGAL included SKELETON APPROACH				  */
		/**************************************************************/

		s->runCGALCode();
		s->computationFinished = true;

	}

	if(config.timings) {
		end = clock();
	}

	write();

	if(config.timings) {
		double time_spent = 0.0 + (double)(end - begin) / CLOCKS_PER_SEC;
		if(config.verbose) {
			LOG(INFO) << "number of vertices: " << data->getPolygon().size();
			LOG(INFO) << "time spent: " << time_spent << " seconds";
			LOG(INFO) << "filename: " << config.fileName;
		} else {
			std::cout << data->getPolygon().size() << "," << time_spent << "," << config.fileName << std::endl;
		}
	}
}

void Monos::write() {
	if(config.outputType != OutputType::NONE && s->computationFinished) {
		s->writeOBJ(config);
		data->addPolyToOBJ(config);
		if(config.verbose) {LOG(INFO) << "output written";}
	}
}


void Monos::reset() {
	wf->reset();
	init();
}

bool Monos::init() {
	if(!config.isValid()) {return false;}

	/* verfify monotonicity -- if required, rotate to assure
	 * that P is x-monotone */
	if(!data->ensureMonotonicity()) {
		if(config.verbose) {LOG(WARNING) << "polygon is not monotone!";}
		exit(1);
		return false;
	}

	/* compute BBox and min/max monotonicity vertices */
	data->bbox = data->computeBoundingBox();

	/* debug */
	if(config.verbose) {LOG(INFO) << "monotonicity line: " << data->monotonicityLine.to_vector();}

	/** input must be monotone */
	wf->ChainDecomposition();
	if(config.verbose) {LOG(INFO) << "chain decomposition done";}

	return true;
}

