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
#include <vector>

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
	if(config.not_x_mon) {
		if(!data->ensureMonotonicity()) {
			if(config.verbose) {LOG(WARNING) << "polygon is not monotone!";}
			return false;
		}
		LOG(WARNING) << "use version from master branch, this is optimized for efficiency. Only x-monotone input!";
		return false;
	} else {
		data->setMonotonicity(Line(ORIGIN, ORIGIN + Vector(1,0)));
	}

	wf = new Wavefront(*data);
	s  = new Skeleton(*data,*wf);

	if(config.duplicate) {
		duplicateInput();
		return false;
	}

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


void Monos::duplicateInput() {
	LOG(INFO) << "-- make " << config.copies << " copies of the input.";
	/* x-monotone, so we offset every point by the x-span of the bbox */
	auto offset = NT(0.4) + CGAL::abs( (data->bbox->xMax.p.x() - data->bbox->xMin.p.x()) );
	auto scale = NT(1.01);

	Vector off(offset,0);

	std::vector<Point> upperRtoL;
	std::vector<Point> lowerLtoR;

	auto leftLowerEdge = data->findEdgeWithVertex(data->bbox->xMin);
	auto it = leftLowerEdge;

	Point pIt;

	/* store the lower chain */
	do {
		pIt = data->v(it->u).p;
		lowerLtoR.emplace_back(pIt);
		it = data->cNext(it);
	} while(it->u != data->bbox->monMax.id);

	pIt = data->v(it->u).p + Vector(0.001,-0.1);
	lowerLtoR.emplace_back(pIt);

	/* store the upper chain */
	do {
		pIt = data->v(it->u).p;
		upperRtoL.emplace_back(pIt);
		it = data->cNext(it);
	} while(it->u != data->bbox->monMin.id);

	pIt = data->v(it->u).p + Vector(0.001,0.1);
	upperRtoL.emplace_back(pIt);
	/* start end point already offset for both chains */



	std::ofstream outfile (config.outputFileName,std::ofstream::binary);
	outfile << "# OBJ-File autogenerated by monos input ("
			<< config.fileName << ") - copied " << config.copies << " times - "
			<< currentTimeStamp() <<  std::endl;

	unsigned long vertex_cnt = 0;

	for(int i = 0; i < config.copies; ++i) {
		auto localScale = i*scale;
		if(i == 0) {localScale = 1;}
		Vector off(i*offset,-0.001);

		for(auto& p : lowerLtoR) {
			Point pOff = p + off;
			double x = (pOff.x()*localScale).doubleValue();
			double y = (pOff.y()*localScale).doubleValue();
			double z = 0;
			outfile << "v " << x << " " << y << " " << z << std::endl;

			++vertex_cnt;
		}
	}

	for(int i = config.copies-1; i >= 0; --i) {
	   auto localScale = i*scale;
		if(i == 0) {localScale = 1;}
		Vector off(i*offset,0.001);

		for(auto& p : upperRtoL) {
			Point pOff = p + off;
			double x = (pOff.x()*localScale).doubleValue();
			double y = (pOff.y()*localScale).doubleValue();
			double z = 0;
			outfile << "v " << x << " " << y << " " << z << std::endl;

			++vertex_cnt;
		}
	}


	outfile << "f";
	for(unsigned long i = 0; i < vertex_cnt; ++i) {
		outfile << " " << i+1;
	}

	outfile << std::endl;
	outfile.close();
}




