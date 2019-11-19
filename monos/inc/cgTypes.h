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

#ifndef CGTYPES_H_
#define CGTYPES_H_

#include <iterator>
#include <array>
#include <algorithm>
#include <functional>
#include <queue>
#include <set>
#include <cmath>

#include "Definitions.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/aff_transformation_tags.h>
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>

using K 			 	= CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;

using Vector         	= K::Vector_2;
using Point          	= K::Point_2;
using Line           	= K::Line_2;
using Ray            	= K::Ray_2;
using Circle         	= K::Circle_2;
using Direction      	= K::Direction_2;
using Segment      	 	= K::Segment_2;
using Intersect		 	= K::Intersect_2;
using Transformation 	= CGAL::Aff_transformation_2<K>;
using NT 	         	= K::FT;

using Chain 			= std::list<ul>;
using ChainRef			= Chain::iterator;
//using PartialSkeleton 	= std::list<ul>;

using PointIterator 	= std::vector<Point,std::allocator<Point>>::const_iterator;

static Point ORIGIN = Point(0,0);
static Point INFPOINT(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());

class Vertex {
public:
	const Point p;
	const unsigned id;

	Vertex(const Point& p, unsigned id)
	: p(p)
	, id(id)
	{}
	friend std::ostream& operator<< (std::ostream& os, const Vertex& vertex);
};

class Edge {
public:
	const unsigned u, v;
	const unsigned id;
	const Segment segment;

	Edge(unsigned u, unsigned v, unsigned id, Segment s)
	: u(u)
	, v(v)
	, id(id)
	, segment(s) {}

	inline bool has(const unsigned idx) const {return u == idx || v == idx;}

	friend std::ostream& operator<< (std::ostream& os, const Edge& edge);
};

using VertexList = std::vector<Vertex>;
using EdgeList = std::vector<Edge>;
using VertexIdxPair = std::pair<unsigned,unsigned>;

class BBox {
public:
	Vertex xMin, xMax, yMin, yMax;
	Vertex monMin, monMax;
};

struct MonotoneVector {
	MonotoneVector(Vector _v, MonotoneType _t, ul _i) :
		vector(_v),type(_t),id(_i) {}
	Vector vector;
	MonotoneType type = {MonotoneType::START};
	ul id = {0};
	friend std::ostream& operator<< (std::ostream& os, const MonotoneVector& mv);
};

struct MonVectCmp {
	bool operator()(const MonotoneVector &first, const MonotoneVector &second) const {
		Point A = ORIGIN + first.vector;
		Point B = ORIGIN + second.vector;

		return   CGAL::left_turn(A,ORIGIN,B) ||
				(CGAL::collinear(A,ORIGIN,B) && A.x() < ORIGIN.x() && B.x() > ORIGIN.x()) ||
				(CGAL::collinear(A,ORIGIN,B) && A.y() < ORIGIN.y() && B.y() > ORIGIN.y()) ||
				(CGAL::collinear(A,ORIGIN,B) &&
						( (A.x() < ORIGIN.x() && B.x() < ORIGIN.x()) ||
						  (A.x() > ORIGIN.x() && B.x() > ORIGIN.x()) ||
						  (A.y() < ORIGIN.y() && B.y() < ORIGIN.y()) ||
						  (A.y() > ORIGIN.y() && B.y() > ORIGIN.y()) ) &&
						first.type == MonotoneType::END);
	}
};


class Bisector {
public:
	Bisector(Ray r,  ul idxA, ul idxB) :type(BisType::RAY), ray(r),  eIdxA(idxA), eIdxB(idxB)  {}
	Bisector(Line l, ul idxA, ul idxB) :type(BisType::LINE),line(l), eIdxA(idxA), eIdxB(idxB) {}

	BisType type;
	Ray     ray;
	Line    line;


	ul eIdxA, eIdxB;

	bool isRay()  const { return type == BisType::RAY; }
	bool isLine() const { return type == BisType::LINE;}

	Direction direction() const  { return (isRay()) ? ray.direction() : line.direction(); }
	Line supporting_line() const { return (isRay()) ? ray.supporting_line() : line; }
	Point point(ul i = 0) const { return supporting_line().point(i); }
	Vector to_vector() const { return (isRay()) ? ray.to_vector() : line.to_vector(); }

	void setRay(const Ray r) {ray = Ray(r); type = BisType::RAY;}

	bool isAA() const {if(isRay()) {return ray.is_vertical() || ray.is_horizontal();} else {return line.is_vertical() || line.is_horizontal();}}
	bool is_vertical() const {if(isRay()) {return ray.is_vertical();} else {return line.is_vertical();}}
	bool is_horizontal() const {if(isRay()) {return ray.is_horizontal();} else {return line.is_horizontal();}}
	void setParallel(bool p) {parallel = p;}
	bool isParallel() const {return parallel;}
	void setGhost(bool g) {ghost = g;}
	bool isGhost() const {return ghost;}

	void changeDirection() {
		if(isRay()) {
			ray = ray.opposite();
		} else {
			line = line.opposite();
		}
	}

	void changeToLine() {
		line = supporting_line();
		type = BisType::LINE;
		ray  = Ray();
	}

	void newSource(const Point s) {
		if(isRay()) {
			ray = Ray(s,direction());
		} else {
			line = Line(s,direction());
		}
	}

private:
	/* true if perpendicular to monotonicity line */
	bool parallel = false;
	bool ghost    = false;

	friend std::ostream& operator<< (std::ostream& os, const Bisector& bis);
};


class TimeEdge {
public:
	TimeEdge(NT t, ul e):
		timeApprox(t.doubleValue()),
		time(t),
		edgeIdx(e) {}
	double timeApprox;
	NT  time;
	ul  edgeIdx;

	inline bool operator==(const TimeEdge& rhs) const {
		return this->timeApprox - rhs.timeApprox <= smallEPS &&
			   rhs.timeApprox - this->timeApprox <= smallEPS &&
			   this->time == rhs.time;
	}
};

struct TimeEdgeCmp {
	bool operator()(const TimeEdge &left, const TimeEdge &right) const {
		return ( (left.timeApprox - right.timeApprox) < 0.0 ) ||
			   ( (left.timeApprox - right.timeApprox) >= 0.0 && left.time < right.time) ||
			   ( (left.timeApprox - right.timeApprox) >= 0.0 && left.time == right.time && left.edgeIdx < right.edgeIdx);
	}
//		return (left.timeApprox < right.timeApprox) ||
//			   (left.timeApprox == right.timeApprox && left.time < right.time) ||
//			   (left.timeApprox == right.timeApprox && left.time == right.time && left.edgeIdx < right.edgeIdx);
//	}
};

using EventTimes 	     = std::set<TimeEdge,TimeEdgeCmp>;

class Event {
public:
	Event(NT time = 0, Point point = INFPOINT, ul edgeA = 0, ul edgeB = 0, ul edgeC = 0, ChainRef ref = ChainRef()):
		eventTime(time),
		eventPoint(point),
		leftEdge(edgeA),
		mainEdge(edgeB),
		rightEdge(edgeC),
		chainEdge(ref) {}

	inline bool isEvent() const { return eventPoint != INFPOINT;}

	NT	         	eventTime;
	Point  			eventPoint;
	ul 				leftEdge, mainEdge, rightEdge;

	ChainRef 		chainEdge;

	inline bool operator==(const Event& rhs) const {
		return this->leftEdge == rhs.leftEdge
			&& this->mainEdge == rhs.mainEdge
			&& this->rightEdge == rhs.rightEdge;
	}
	inline bool operator!=(const Event& rhs) const {
		return !(*this == rhs);
	}
	friend std::ostream& operator<< (std::ostream& os, const Event& event);
};

using Events		     = std::vector<Event>;

class Arc {
public:
	Arc(ArcType t, ul firstNode, ul leftEdge, ul rightEdge, unsigned id, Ray r):
		type(t), firstNodeIdx(firstNode), secondNodeIdx(MAX),
		leftEdgeIdx(leftEdge), rightEdgeIdx(rightEdge), id(id),
		segment(Segment()),ray(r) {}
	Arc(ArcType t, ul firstNode, ul secondNode, ul leftEdge, ul rightEdge, unsigned id, Segment e):
		type(t), firstNodeIdx(firstNode), secondNodeIdx(secondNode),
		leftEdgeIdx(leftEdge), rightEdgeIdx(rightEdge), id(id),
		segment(e),ray(Ray()) {}

	inline bool isAA() const {
		return is_vertical() || is_horizontal();
	}

	inline bool has_on(const Point& p) const {
		if(isEdge()) {
			return segment.has_on(p);
		} else {
			return ray.has_on(p);
		}
//		if(is_vertical() && p.x() != point(0).x()) {
//			auto yMin = point(0).y();
//			auto yMax = point(1).y();
//			if(yMin > yMax) {std::swap(yMin,yMax);}
//
//			if( p.y() < yMin || p.y() > yMax || p.x() != point(0).x()) {
//				return false;
//			}
//		}
//		if(is_horizontal()) {
//			auto xMin = point(0).x();
//			auto xMax = point(1).x();
//			if(xMin > xMax) {std::swap(xMin,xMax);}
//
//			if( p.x() < xMin || p.x() > xMax || p.y() != point(0).y()) {
//				return false;
//			}
//		}
//		if(isEdge()) {
//			return edge.has_on(p);
//		} else {
//			return ray.has_on(p);
//		}
	}

	inline bool hasZeroLength() const { return (firstNodeIdx == secondNodeIdx);}

	ul getCommonNodeIdx(const Arc& arc) {
		if(firstNodeIdx == arc.firstNodeIdx || firstNodeIdx == arc.secondNodeIdx) {
			return firstNodeIdx;
		}
		if(secondNodeIdx == arc.firstNodeIdx || secondNodeIdx == arc.secondNodeIdx) {
			return secondNodeIdx;
		}
		return MAX;
	}

	bool isParallel(const Arc& arc) const {
		return !isDisable() && CGAL::parallel(supporting_line(),arc.supporting_line());
	}
	bool isCollinear(const Arc& arc) const {
		return !isDisable() && CGAL::collinear(point(0),point(1), arc.point(0) + arc.to_vector());
	}

	bool adjacent(const Arc& arc) const {
		return firstNodeIdx  == arc.firstNodeIdx || firstNodeIdx  == arc.secondNodeIdx ||
			   secondNodeIdx == arc.firstNodeIdx || secondNodeIdx == arc.secondNodeIdx;
	}
	bool is_vertical()   const { return (isEdge()) ? segment.is_vertical()   : ray.is_vertical();}
	bool is_horizontal() const { return (isEdge()) ? segment.is_horizontal() : ray.is_horizontal();}

	ul getSecondNodeIdx(const ul idx) const { return (idx == firstNodeIdx) ? secondNodeIdx : firstNodeIdx; }

	std::vector<ul> getNodeIndices() {
		if(isEdge()) {
			return {firstNodeIdx,secondNodeIdx};
		} else {
			return {firstNodeIdx};
		}
	}

	Line supporting_line() const {return (isEdge()) ? segment.supporting_line() : ray.supporting_line();}
	Vector to_vector() const {return supporting_line().to_vector();}
	inline bool isRay()  const { return type == ArcType::RAY;    }
	inline bool isEdge() const { return type == ArcType::NORMAL; }
	inline bool isDisable() const {return type == ArcType::DISABLED;}
	inline void disable() {type = ArcType::DISABLED;}

	Point point(int i) const {return (isEdge()) ? segment.vertex(i) : ray.point(i);}

	bool hasEndPoint(const Point& P) const {
		LOG(INFO) << "hasEndPoint";
		return P == point(0) || (isEdge() && P == point(1));
	}

	ArcType type;
	ul firstNodeIdx, secondNodeIdx;
	ul leftEdgeIdx,  rightEdgeIdx;
	unsigned id;

	Segment segment;
	Ray  ray;

	friend std::ostream& operator<< (std::ostream& os, const Arc& arc);
};

using ArcList		= std::vector<Arc>;

struct ArcCmp {
	ArcCmp(const ArcList& list):arcList(list) {}
	const ArcList& arcList;
	bool operator()(const ul &left, const ul &right) const {
		auto leftArc  = &(arcList)[left];
		auto rightArc = &(arcList)[right];
		return (   (leftArc->firstNodeIdx  == rightArc->firstNodeIdx  || leftArc->secondNodeIdx  == rightArc->secondNodeIdx) && leftArc->rightEdgeIdx == rightArc->leftEdgeIdx)
				|| (leftArc->firstNodeIdx  == rightArc->secondNodeIdx && leftArc->rightEdgeIdx   == rightArc->rightEdgeIdx)
				|| (leftArc->secondNodeIdx == rightArc->firstNodeIdx  && leftArc->leftEdgeIdx    == rightArc->leftEdgeIdx);
	}
};


struct Node {
	Node(const NodeType t, const Point p, NT time, unsigned id): type(t), point(p), time(time), id(id) {}

	NodeType 		type;
	Point			point;
	NT				time;
	unsigned		id;
	bool			ghost = false;

	/* all incident arcs, i.e., the indices to them */
	std::vector<ul> 	arcs;

	void disable() {type = NodeType::DISABLED;}
	bool isDisabled() const { return type == NodeType::DISABLED;}
	bool isTerminal() const { return type == NodeType::TERMINAL;}

	bool isGhostNode() const { return ghost; }
	void setGhost(bool g) {ghost = g;}

	ul degree() const {return arcs.size();}

	void sort(const ArcList& arcList) {
		std::sort(arcs.begin(), arcs.end(), ArcCmp(arcList));
	}

	bool hasArc(const ul& arcIdx) const {
		auto idx = std::find(arcs.begin(),arcs.end(),arcIdx);
		return idx != arcs.end();
	}

	bool removeArc(const ul& arcIdx) {
		auto idx = std::find(arcs.begin(),arcs.end(),arcIdx);
		if(idx != arcs.end()) {
			arcs.erase(idx);
			return true;
		}
		return false;
	}

	friend std::ostream& operator<< (std::ostream& os, const Node& node);
};

class EndNodes {
public:
	EndNodes(sl a_ = NIL, sl b_ = NIL):
		a(a_),
		b(b_) {}
	sl a, b;
};

/* the inputPoints index equals the terminal node index,
 * as from every vertex emits an arc */
using Nodes 		= std::vector<Node>;
using PathFinder    = std::vector<EndNodes>;

NT normalDistance(const Line& l, const Point& p);

template<class T, class U>
Point intersectElements(const T& a, const U& b);

template<class T, class U>
bool isLinesParallel(const T& a, const U& b);

template<class T, class U>
Point intersectElements(const T& a, const U& b) {
//	Point intersectionPoint = INFPOINT;

//	LOG(INFO) << "(" << a << " -- " << b << ") ";

//	if(CGAL::do_intersect(a,b)) {
		auto result = CGAL::intersection(a, b);
		if (result) {
			if (const Point* p = boost::get<Point>(&*result)) {
				return Point(*p);
			} else if (const Segment* e = boost::get<Segment>(&*result)) {
//				LOG(INFO) << "# Intersection forms a segment - returning edge-point(0)";
				return Point(e->point(0));
//			} else {
//				LOG(WARNING) << "# Intersection forms a something else?!? - returning INFPOINT";
//				return INFPOINT;
			}
		}
//	}
	return INFPOINT;
}

ul getArcsCommonNodeIdx(const Arc& arcA, const Arc& arcB);

Point intersectArcArc(const Arc& arcA, const Arc& arcB);
Point intersectRayArc(const Ray& ray, const Arc& arc);
Point intersectBisectorEdge(const Bisector& bis, const Segment& edge);

template<class T, class U>
bool isLinesParallel(const T& a, const U& b) {
	return CGAL::parallel(Line(a),Line(b));
}

bool do_intersect(const Bisector& ray, const Arc& arc);
bool do_intersect(const Bisector& ray, const Segment& edge);

#endif /* CGALTYPES_H_ */
