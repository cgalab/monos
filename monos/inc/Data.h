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

#include "BasicInput.h"


class Data {
	using EdgeIterator = EdgeList::const_iterator;
	using VertexIterator = VertexList::const_iterator;

public:
	Data(const BasicInput& input_):
		input(input_) {}

	~Data() {delete bbox;}

	const VertexList& getVertices() const { return input.vertices(); }
	const EdgeList&   getPolygon()  const { return input.edges();    }

	inline const Edge e(const ul& idx) const { return input.get_edge(idx); }
	const Vertex& v(const ul& idx) const { return getVertices()[idx]; }
	const Point& p(const ul& idx) const { return v(idx).p; }
	inline Line get_line(const ul& idx) const {return e(idx).line;}
	inline Line get_line(const EdgeIterator& it) const {return it->line;}
	Segment get_segment(const ul& idx) const {return e(idx).segment;}
	Segment get_segment(const EdgeIterator& it) const {return it->segment;}

	const Point& eA(const ul& edgeIdx) const {return p(e(edgeIdx).u);}
	const Point& eB(const ul& edgeIdx) const {return p(e(edgeIdx).v);}

	inline NT normalDistance(const ul& edgeIdx, const Point& p) const {
		return CGAL::squared_distance(get_line(edgeIdx),p);
	}

	inline Line simpleBisector(const ul& a, const ul& b) {
		return CGAL::bisector(get_line(a),get_line(b).opposite());
	}

	/* verify if the input polygon is monotone, if required we rotate
	 * the vertices such that x-monotonicity holds for P */
	bool ensureMonotonicity();
	bool isAbove(const Point& a, const Point &b) const;
//	inline bool monotoneSmaller(const Point& a, const Point& b) const {
//		return a < b ; //Line(b,perpMonotonDir).has_on_positive_side(a);
//	}
//	inline bool monotoneSmaller(const Line& line, const Point& a, const Point& b) const {
//		return a < b; //line.perpendicular(b).has_on_positive_side(a);
//	}

	void setMonotonicity(Line line) {
		monotonicityLine = line;
		perpMonotonDir = monotonicityLine.direction().perpendicular(CGAL::POSITIVE);
		assignBoundingBox();
	}

	Line			monotonicityLine;
	Direction		perpMonotonDir;

	BBox			*bbox = nullptr;

	EdgeIterator findEdgeWithVertex(const Vertex& v) const {
		for(auto eit = getPolygon().begin(); eit != getPolygon().end(); ++eit) {
			if(eit->u == v.id) {return eit;}
		}
		return getPolygon().end();
	}


	inline EdgeIterator cNext(EdgeIterator it) {return (std::next(it) == getPolygon().end()) ? getPolygon().begin(): std::next(it);}
	EdgeIterator cPrev(EdgeIterator it) {return (it == getPolygon().begin()) ? std::prev(getPolygon().end()) : std::prev(it);}

	/* write output & debug misc */
	void addPolyToOBJ(const Config& cfg) const;
	void printInput() const;
	void printLineFormat();
private:
	void assignBoundingBox();

	Line getMonotonicityLineFromVector(const Vector a, const Vector b) const;
	bool testMonotonicityLineOnPolygon(const Line line) const;

	const BasicInput& 	input;
};

#endif /* DATA_H_ */
