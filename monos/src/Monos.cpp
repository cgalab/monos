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

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <exception>

#include <sys/resource.h>

#include "Monos.h"
#include "Data.h"
#include "BGLGraph.h"
#include "BasicInput.h"

#include "EventQueue.h"
#include <random>
Monos::Monos(const Config& cfg):config(cfg) {}

Monos::~Monos() {
	delete data;
	delete wf;
	delete s;
}


bool Monos::readInput() {
	std::ifstream in;
	if(fileExists(config.fileName)) {
		in.open(config.fileName);
		BGLGraph gml = BGLGraph::create_from_graphml(in);
		input = BasicInput();
		input.add_graph(gml);
		return true;
	}
	return false;
}


void Monos::run() {
	clock_t begin, end;

//	/**************************************************************/
//	/*				TESTING THE HEAP							  */
//	/**************************************************************/
//
//

//	std::vector<Event> bla;
//	Chain c;
//	for(int i = 0; i < 100; ++i) {
//		c.push_back(i+5);
//		bla.emplace_back(Event(NT(i),INFPOINT,i,i+1,i+2));
//	}
//
//
//
//	std::default_random_engine rng = std::default_random_engine {};
//	std::shuffle(std::begin(bla), std::end(bla), rng);
//
//	Events eve;
//	eve.resize(106,Event());
//	for(int i = 0; i < 100; ++i) {
//		eve[i] = bla[i];
//		eve[i].mainEdge = i+5;
//	}
//
//	LOG(INFO) << "now the queue:";
//
//	EventQueue myQueue(eve,c);
//
//	while(!myQueue.empty()) {
//		auto eidx = myQueue.peak()->idx_in_heap;
//		auto e = myQueue.peak()->priority;
//		std::cout << e.e->mainEdge << " heap idx: " << eidx << std::endl;
//		myQueue.pop();
//	}
//
//	return;
//	/**************************************************************/
//	/*				END TESTING THE HEP      					  */
//	/**************************************************************/


	if(!readInput()) {return;}

	/****************** TIMING START ******************************/
	if(config.timings) {begin = clock();}

	/**************************************************************/
	/*				MONOTONE SKELETON APPROACH 					  */
	/**************************************************************/

	if(!init()) {return;}

	if(!wf->ComputeSkeleton(ChainType::LOWER)) {return;}
	if(config.verbose) {LOG(INFO) << "lower skeleton done";}

	if(!wf->ComputeSkeleton(ChainType::UPPER)) {return;}
	if(config.verbose) {LOG(INFO) << "upper skeleton done";}

	s->MergeUpperLowerSkeleton();
	if(config.verbose) {LOG(INFO) << "merging upper and lower skeleton done";}


	/****************** TIMING END ********************************/
	if(config.timings) {end = clock();}

	write();

	if(config.timings) {
		struct rusage usage;
		if (getrusage(RUSAGE_SELF, &usage) < 0) {
			LOG(ERROR) << "getrusage() failed: " << strerror(errno);
			exit(1);
		}
		double time_spent = 0.0 + (double)(end - begin) / CLOCKS_PER_SEC;
		if(config.verbose) {
			LOG(INFO) << "number of vertices: " << data->getPolygon().size();
			LOG(INFO) << "time spent: " << time_spent << " seconds";
			LOG(INFO) << "mem usage : " << usage.ru_maxrss << " kB";
			LOG(INFO) << "filename: " << config.fileName;
		} else {
			std::cout << data->getPolygon().size()
					  << "," << time_spent
					  << "," << usage.ru_maxrss
					  << "," << config.fileName
					  << std::endl;
		}
	}
}

void Monos::write() {
	if( s->computationFinished ) {
		s->writeOBJ(config);
		data->addPolyToOBJ(config);
		if(config.verbose) {LOG(INFO) << "output written";}
	}
}


bool Monos::init() {
	data = new Data(input);

	/* verify monotonicity and compute monotonicity line */
	if(config.x_mon) {
		data->setMonotonicity(Line(ORIGIN, ORIGIN + Vector(1,0)));
	} else {
		if(!data->ensureMonotonicity()) {
			if(config.verbose) {LOG(WARNING) << "polygon is not monotone!";}
			return false;
		}
	}

	wf = new Wavefront(*data);
	s  = new Skeleton(*data,*wf);

	/* debug */
	if(config.verbose) {LOG(INFO) << "monotonicity line: " << data->monotonicityLine.to_vector();}

	/** input must be monotone */
	wf->ChainDecomposition();
	if(config.verbose) {LOG(INFO) << "chain decomposition done";}
	wf->printChain(wf->getChain(ChainType::UPPER));
	wf->printChain(wf->getChain(ChainType::LOWER));

	s->storeChains(wf->getChain(ChainType::UPPER), wf->getChain(ChainType::LOWER));

	/* initialize wavefront and skeleton */
	wf->InitializeNodes();
	wf->InitializeEventsAndPathsPerEdge();

	return true;
}

