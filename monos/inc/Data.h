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


#ifndef DATA_H_
#define DATA_H_

#include <memory>
#include <assert.h>
#include <ctime>
#include <fstream>

#include <iterator>
#include <algorithm>

#include <sys/stat.h>

#include <vector>
#include <list>
#include <set>
#include <string>

#include "cgTypes.h"
#include "Config.h"
#include "Definitions.h"

enum class MonotoneType    : uint {START=0,END};

struct MonotoneVector {
	MonotoneVector(Vector _v, MonotoneType _t, uint _i) :
		vector(_v),type(_t),id(_i) {}
	Vector vector;
	MonotoneType type = {MonotoneType::START};
	uint id = {0};
	friend std::ostream& operator<< (std::ostream& os, const MonotoneVector& mv);
};

struct MonVectCmp {
	bool operator()(const MonotoneVector &left, const MonotoneVector &right) const {
		return CGAL::left_turn(ORIGIN-left.vector,ORIGIN,ORIGIN+right.vector);
	}
};


class Data {
	using EdgeIterator = std::vector<IndexEdge,CORE::allocator<IndexEdge>>::iterator;

public:
	Data()  {}
	~Data() {}

	BBox bbox;

	void initialize(const Config& cfg);
	BBox computeBoundingBox() const;

	const InputPoints&  getVertices() const { return inputVertices; }
	const Polygon&      getPolygon()  const { return polygon; }
	const InputWeights& getWeights()  const { return edgeWeights; }

	const IndexEdge e(const uint& idx) const { assert(idx < polygon.size() ); return polygon[idx]; }
	const Point& v(const uint& idx) const { assert(idx < inputVertices.size()); return inputVertices[idx]; }
	const Exact w(const uint& idx) const { assert(idx < inputVertices.size()); return edgeWeights[idx]; }
	Edge getEdge(const uint& idx) const;
	Edge getEdge(const EdgeIterator& it) const;

	const Point& eA(const uint& edgeIdx) const {return v(e(edgeIdx)[0]);}
	const Point& eB(const uint& edgeIdx) const {return v(e(edgeIdx)[1]);}

	Exact normalDistance(const uint& edgeIdx, const Point& p) const {
		Line l(getEdge(edgeIdx));
		return CGAL::squared_distance(l,p);
	}

	bool isEdgeCollinear(const uint& i, const uint& j) const;
	Edge confineRayToBBox(const Ray& ray) const;

	void addPolyToOBJ(const Config& cfg) const;
	void printInput() const;

	/* verify if the input polygon is monotone, if required we rotate
	 * the vertices such that x-monotonicity holds for P */
	bool ensureMonotonicity();
	bool monotoneSmaller(const Point& a, const Point& b) const;

	const BasicInput& getBasicInput() const {return basicInput;}

	void setGui(const bool _gui) { gui = _gui; }

private:
	bool loadFile(const std::string& fileName);

	bool parseOBJ(const std::vector<std::string>& lines);
	bool parseGML(std::istream &istream);
	bool parsePOLY(const std::vector<std::string>& lines);
	/** Input: vertices as 2D coordinates, polygon as edge "list"
	 *  with index to the inputVertices
	 **/
	InputPoints		inputVertices;
	Polygon 		polygon;
	/* for every edge in polygon we store a strictly positive edge weight */
	InputWeights	edgeWeights;

	Line			monotonicityLine;

	GMLGraph		gml;
	BasicInput		basicInput;

	bool 			gui = false;
};

#endif /* DATA_H_ */
