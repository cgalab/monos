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

	const Edge e(const ul& idx) const { return input.get_edge(idx); }
	const Vertex& v(const ul& idx) const { return getVertices()[idx]; }
	const Point& p(const ul& idx) const { return v(idx).p; }
	Segment get_segment(const ul& idx) const {return e(idx).segment;}
	Segment get_segment(const EdgeIterator& it) const {return it->segment;}

	const Point& eA(const ul& edgeIdx) const {return p(e(edgeIdx).u);}
	const Point& eB(const ul& edgeIdx) const {return p(e(edgeIdx).v);}

	NT normalDistance(const ul& edgeIdx, const Point& p) const {
		Line l(get_segment(edgeIdx));
		return CGAL::squared_distance(l,p);
	}


	bool isEdgeCollinear(const ul& i, const ul& j) const;
	bool isEdgeCollinear(const Segment& eA, const Segment& eB) const;
	bool isEdgeCollinearAndInteriorRight(const ul& i, const ul& j) const;
	bool isEdgeCollinearAndInteriorLeft(const ul& i, const ul& j) const;
	bool isEdgeCollinearAndCommonInteriorDirection(const ul& i, const ul& j) const;
	bool isEdgesParallel(const ul& i, const ul& j) const {
		return isLinesParallel(get_segment(i).supporting_line(),get_segment(j).supporting_line());
	}

	inline Line simpleBisector(const ul& a, const ul& b) {
		return CGAL::bisector(get_segment(a).supporting_line(),get_segment(b).supporting_line().opposite());
	}

	/* verify if the input polygon is monotone, if required we rotate
	 * the vertices such that x-monotonicity holds for P */
	bool ensureMonotonicity();
	bool isAbove(const Point& a, const Point &b) const;
	bool monotoneSmaller(const Point& a, const Point& b) const;
	bool monotoneSmaller(const Line& line, const Point& a, const Point& b) const;
	bool rayPointsLeft(const Ray& ray) const;
	Point pointOnMonotonicityLine(const Point p) const { return monotonicityLine.projection(p); }

	bool pointsEqualIfProjectedToMonotonicityLine(const Point a, const Point b) const {
		return pointOnMonotonicityLine(a) == pointOnMonotonicityLine(b);
	}

	void setMonotonicity(Line line) {
		monotonicityLine = line;
		perpMonotonDir = monotonicityLine.direction().perpendicular(CGAL::POSITIVE);
		isMonotone = true;
		assignBoundingBox();
	}

	Line			monotonicityLine;
	Direction		perpMonotonDir;
	bool			isMonotone = false;

	BBox			*bbox = nullptr;


	std::vector<Segment> lines;
//	void visualizeBisector(Segment edge) {
//		if(gui) {
//			if(!lines.empty()) {lines.pop_back();}
//			lines.push_back(edge);
//		}
//	}

	EdgeIterator findEdgeWithVertex(const Vertex& v) const {
		for(auto eit = getPolygon().begin(); eit != getPolygon().end(); ++eit) {
			if(eit->u == v.id) {
				return eit;
			}
		}
		return getPolygon().end();
	}


	EdgeIterator cNext(EdgeIterator it) {return (std::next(it) == getPolygon().end()) ? getPolygon().begin(): std::next(it);}
	EdgeIterator cPrev(EdgeIterator it) {return (it == getPolygon().begin()) ? std::prev(getPolygon().end()) : std::prev(it);}

	/* write output & debug misc */
	void addPolyToOBJ(const Config& cfg) const;
	void printInput() const;

private:
	void assignBoundingBox();

	Line getMonotonicityLineFromVector(const Vector a, const Vector b) const;
	bool testMonotonicityLineOnPolygon(const Line line) const;

//	const bool 			gui;
	const BasicInput& 	input;
};

#endif /* DATA_H_ */
