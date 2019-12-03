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

#ifndef MONOS_H_
#define MONOS_H_

#include <list>

#include "Config.h"
#include "Data.h"
#include "cgTypes.h"
#include "Definitions.h"
#include "BasicInput.h"

#include "Wavefront.h"
#include "Skeleton.h"

class Monos {
public:
	Monos(const Config& cfg);
	~Monos();

	/**
	 * a full run of monos
	 *  */
	void run();

	/**
	 * verify valid config, initialize BBox, chain decomp.
	 * */
	bool readInput();
	bool init();
	void write();

	/* only to produce big inputs */
	void duplicateInput();
	Point offsetPoint(Point p, NT eps) {return Point(p.x(),p.y()+ p.y()*eps);} // + eps*100.0/(1+(rand()%100)));}

	const Config&   config;

	const BasicInput* getBasicInput() {return &input;}

	Data			*data 	= nullptr;
	Wavefront 		*wf 	= nullptr;
	Skeleton		*s 		= nullptr;

	BasicInput		input;
private:
};

#endif /* MONOS_H_ */
