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

	write();
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
//	Point p = data->v(data->bbox.monotoneMinIdx);
//	Edge e(p,p+data->monotonicityLine.to_vector()); // = data->confineRayToBBox(Ray(p,data->monotonicityLine.to_vector()));
//	data->lines.push_back(e);

	/* DEBUG TESTING !REMOVE! */
//	minimumExample();

	/** input must be monotone */
	wf->ChainDecomposition();
	if(config.verbose) {LOG(INFO) << "chain decomposition done";}

	return true;
}

//Point intersect(Line a, Line b) {
//	auto result = CGAL::intersection(a,b);
//	if (result) {
//		if (const Point* p = boost::get<Point>(&*result)) {
//			return Point(*p);
//		}
//	}
//	return INFPOINT;
//}
//
//void Monos::minimumExample() {
//	LOG(INFO) << "#################### MINIMUM EXAMPLE! ##########################";
//	Point Pa1(0,0), Pa2(0,4), Pb1(4,0), Pb2(4,4), Pc1(-2,6), Pc2(1,6);
//	Edge a(Pa2,Pa1), b(Pb1,Pb2), c(Pc1,Pc2);
//	Edge br(Pa2,Pb2), br2(Pa1,Pb1);
//
//	auto bisAB = CGAL::bisector(a.supporting_line(),b.supporting_line());
//	auto bisAC = CGAL::bisector(a.supporting_line(),c.supporting_line());
//
//	LOG(INFO) << "bisAB " << bisAB << " dir: " << bisAB.to_vector();
//	LOG(INFO) << "bisAC " << bisAC << " dir: " << bisAC.to_vector();
//
//	Point PBisABa = intersect(bisAB,br2.supporting_line());
//	Point PBisABb = intersect(bisAB,br.supporting_line());
//	Point PBisACa = intersect(bisAC,c.supporting_line());
//	Point PBisACb = intersect(bisAC,br.supporting_line());
//
//	LOG(INFO) << "bisABEd " << PBisABa << " -- "<< PBisABb;
//	LOG(INFO) << "bisACEd " << PBisACa << " -- "<< PBisACb;
//
//	Edge bisEdgeAB(PBisABa,PBisABb);
//	Edge bisEdgeAC(PBisACa,PBisACb);
//
//	if(CGAL::do_intersect(bisEdgeAB,bisEdgeAC)) {
//		auto result = CGAL::intersection(bisEdgeAB,bisEdgeAC);
//		if (result) {
//			if (const Point* p = boost::get<Point>(&*result)) {
//				Point inters = Point(*p);
//				LOG(INFO) << inters;
//			}
//		}
//	}
//
//	LOG(INFO) << "#################### END MINIMUM EXAMPLE! ##########################";
//
//}

