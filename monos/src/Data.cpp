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


#include "Data.h"
#include "tools.h"

std::ostream& operator<< (std::ostream& os, const MonotoneVector& mv) {
	os << mv.vector << " id(" << mv.id << ")";
	return os;
}

bool Data::isEdgeCollinear(const Segment& eA, const Segment& eB) const {
	return CGAL::parallel(eA,eB) && CGAL::collinear(eA.point(0),eA.point(1),eB.point(0));
}

bool Data::isEdgeCollinear(const ul& i, const ul& j) const {
	return isEdgeCollinear(get_segment(i),get_segment(j));
}

bool Data::isEdgeCollinearAndCommonInteriorDirection(const ul& i, const ul& j) const {
	if(isEdgeCollinear(i,j)) {
		auto lI = get_segment(i).supporting_line();
		auto lJ = get_segment(j).supporting_line();
		auto nEI = lI.perpendicular(lI.point(0)).direction();
		auto nEJ = lJ.perpendicular(lJ.point(0)).direction();
		return nEI == nEJ;
	}
	return false;
}
bool Data::isEdgeCollinearAndInteriorRight(const ul& i, const ul& j) const {
	if(isEdgeCollinear(i,j)) {
		auto lI = get_segment(i).supporting_line();
		auto lJ = get_segment(j).supporting_line();
		auto nEI = lI.perpendicular(lI.point(0)).direction();
		auto nEJ = lJ.perpendicular(lJ.point(0)).direction();
		return nEI == nEJ && nEJ == monotonicityLine.direction();
	}
	return false;
}
bool Data::isEdgeCollinearAndInteriorLeft(const ul& i, const ul& j) const {
	if(isEdgeCollinear(i,j)) {
		auto lI = get_segment(i).supporting_line();
		auto lJ = get_segment(j).supporting_line();
		auto nEI = lI.perpendicular(lI.point(0)).direction();
		auto nEJ = lJ.perpendicular(lJ.point(0)).direction();
		return nEI == nEJ && nEJ == monotonicityLine.opposite().direction();
	}
	return false;
}



bool Data::ensureMonotonicity() {
	assert(input.edges().size() > 2);

	auto edgeIt = input.edges().begin();
	auto edgeB  = edgeIt;
	auto edgeA  = edgeB;

	Vector vA, vB, intervalA, intervalB;
	Point corner;

	vB 		= input.get_segment(*edgeIt).to_vector();
	++edgeIt;


	intervalA = vA;
	intervalB = vA;

	std::set<MonotoneVector,MonVectCmp> intervals;
	ul idCnt = 0;

	/* iterate over polygon and obtaining the 'monotonicity angle'
	 * if it exists. */
	do {
		edgeA = edgeB;
		edgeB = edgeIt;
		vA = vB;
		vB = input.get_segment(*edgeIt).to_vector();
		corner = v(edgeA->v).p;
		/* ensure the vertex is reflex */
		if(CGAL::right_turn(corner-vA,corner,corner+vB)) {
			MonotoneVector a(vA,MonotoneType::START, idCnt);
			MonotoneVector b(vB,MonotoneType::END,   idCnt);
			intervals.insert(a);
			intervals.insert(b);

			a = MonotoneVector(Vector(-vA.x(),-vA.y()),MonotoneType::START,  idCnt+1);
			b = MonotoneVector(Vector(-vB.x(),-vB.y()),MonotoneType::END,    idCnt+1);
			intervals.insert(a);
			intervals.insert(b);

			idCnt+=2;
		}
	} while(++edgeIt != input.edges().end());

	if(intervals.empty()) {
		/* polygon is convex, let us choose the x-axis */
		monotonicityLine = Line(ORIGIN, ORIGIN + Vector(1,0));
		perpMonotonDir = monotonicityLine.direction().perpendicular(CGAL::POSITIVE);
		isMonotone = true;
		return true;
	}

	std::stringstream ss;
	for(auto i : intervals) {
		ss << i << std::endl;
	}
	LOG(INFO) << ss.str();

	/* iterate to first START vector */
	auto itStart  = intervals.begin();
	auto it 	  = itStart;
	int activeCnt = 0;		/* keep track how many intervals are active */
	std::vector<bool> activeIntervals(intervals.size(),false);

	for(;it != intervals.end(); ++it) {
		if(it->type == MonotoneType::END) {
			if(activeIntervals[it->id]) {
				activeIntervals[it->id] = false;
				--activeCnt;
			}
		}
		if(it->type == MonotoneType::START) {
			if(!activeIntervals[it->id]) {
				activeIntervals[it->id] = true;
				++activeCnt;
			}
		}
	}

	it 			    = itStart;
	bool success 	= false;

	if(activeCnt == 0) {
		success = true;
		Vector a = intervals.rbegin()->vector;
		Vector b = intervals.begin()->vector;
		Line line = getMonotonicityLineFromVector(a,b);
		if(testMonotonicityLineOnPolygon(line)) {
			monotonicityLine = line;
			perpMonotonDir   = monotonicityLine.direction().perpendicular(CGAL::POSITIVE);
			isMonotone = true;
		} else {
			assert(false);
		}
	} else {
		do {
			if(it->type == MonotoneType::END) {
				if(activeIntervals[it->id]) {
					activeIntervals[it->id] = false;
					--activeCnt;
				}
			}
			/* we found a window*/
			if(activeCnt == 0) {
				/* TODO VERIFY CORRECT LINE! */
				success = true;
			} else if(it->type == MonotoneType::START) {
				if(!activeIntervals[it->id]) {
					activeIntervals[it->id] = true;
					++activeCnt;
				}
			}

			if(success) {
				/* in case of a rectilinear monotone polygon we have two possible monotonoicity lines
				 * due to the angle intervals we stored, therfore we have to check if we found the  right
				 * one */
				Vector a = it->vector;
				++it;
				Vector b = it->vector;
				Line line = getMonotonicityLineFromVector(a,b);

				LOG(INFO) << "monotonicity Line " << line << " dir: " << line.direction().to_vector() << " found ... testing.";

				if(testMonotonicityLineOnPolygon(line)) {
					monotonicityLine = line;
					perpMonotonDir   = monotonicityLine.direction().perpendicular(CGAL::POSITIVE);
					isMonotone = true;
				} else {
					success = false;
				}
			}

			/* iterate */
			if(++it == intervals.end()) {it = intervals.begin();}

		} while(!success && it != itStart);
	}

	/* construct the monotonicity line */
	if(!success) {
		LOG(WARNING) << "Polygon not monotone!";
		return false;
	}

	assignBoundingBox();

	return true;
}

/* test if given line is a line where the input polygon is monotone to */
bool Data::testMonotonicityLineOnPolygon(const Line line) const {
	ul startIdx = 0;
	Point pStart  = eA(startIdx);
	for(ul i = startIdx+1; i < input.edges().size(); ++i) {
		auto p = eA(i);
		if(monotoneSmaller(line,p,pStart)) {
			startIdx = i; pStart = p;
		}
	}

	/* startIdx is 'leftmost' edge, start walking 'rightwards' until we violate the monotonicity */
	bool monotone  = true, rightward = true;
	auto dir = line.direction().perpendicular(CGAL::POSITIVE);
	auto idxIt = startIdx;
	do {
//		LOG(INFO) << "pa " << eA(idxIt) << ", pb " << eB(idxIt);
		auto testLine = Line(eA(idxIt),-dir);
		if(rightward && testLine.has_on_negative_side(eB(idxIt))) {
//			LOG(INFO) << "end rightward";
			rightward = false;
		} else if(!rightward && testLine.has_on_positive_side(eB(idxIt))) {
//			LOG(INFO) << "end monotone";
			monotone = false;
		}

		if(++idxIt >= input.edges().size()) {idxIt = 0;}
	} while(monotone && idxIt != startIdx);

	return monotone;
}

Line Data::getMonotonicityLineFromVector(const Vector a, const Vector b) const {
	auto c = CGAL::bisector(Line(ORIGIN,a),Line(ORIGIN,b));
	Line l = c.perpendicular(ORIGIN);

	if(l.to_vector().x() < 0.0) {
		l = l.opposite();
	} else if(l.is_vertical() && l.to_vector().y() < 0.0) {
		l = l.opposite();
	}

	return l;
}

bool Data::monotoneSmaller(const Line& line, const Point& a, const Point& b) const {
	assert(a != b);
	auto checkDir = line.direction().perpendicular(CGAL::POSITIVE);
	Line perpL = Line(b,checkDir);
	return perpL.has_on_positive_side(a);
}

bool Data::monotoneSmaller(const Point& a, const Point& b) const {
	assert(a != b);
	Line perpL = Line(b,perpMonotonDir);

	if(perpL.has_on_positive_side(a)) {
		return true;
	} else if(perpL.has_on_negative_side(a)) {
		return false;
	} else {
		Line L = Line(a,monotonicityLine.direction());
		return L.has_on_positive_side(b);
	}
	return false;
}
bool Data::rayPointsLeft(const Ray& ray) const {
	Point Pa = monotonicityLine.point(0);
	Point Pb = Pa + ray.to_vector();
	return monotoneSmaller(Pb,Pa);
}

void Data::assignBoundingBox() {
	auto xMin   = getVertices().begin();
	auto xMax   = getVertices().begin();
	auto yMin   = getVertices().begin();
	auto yMax   = getVertices().begin();
	auto monMin = getVertices().begin();
	auto monMax = getVertices().begin();

	for(auto v = getVertices().begin(); v != getVertices().end(); ++v ) {
		if(v->id != xMin->id && v->p.x() < xMin->p.x()) {xMin = v;}
		if(v->id != xMax->id && v->p.x() > xMax->p.x()) {xMax = v;}
		if(v->id != yMin->id && v->p.y() < yMin->p.y()) {yMin = v;}
		if(v->id != yMax->id && v->p.y() > yMax->p.y()) {yMax = v;}

		if(v->id != monMin->id && monotoneSmaller(v->p,monMin->p)) {monMin = v;}
		if(v->id != monMax->id && monotoneSmaller(monMax->p,v->p)) {monMax = v;}
	}

	bbox = new BBox {
			{xMin->p, xMin->id},
			{xMax->p, xMax->id},
			{yMin->p, yMin->id},
			{yMax->p, yMax->id},
			{monMin->p, monMin->id},
			{monMax->p, monMax->id}
	};

	LOG(INFO)<< "monmin: " << *monMin << ", monMax: "  << *monMax;
}


void Data::printInput() const {
	std::stringstream ss;
	ss << "Input Vertices: " << std::endl;
	for(auto v : input.vertices()) {
		ss << "(" << v.p.x() << "," << v.p.y() << ") ";
	}
	ss << std::endl << "Input Polygon: " << std::endl;
	ul cnt = 0;
	for(auto edge : input.edges()) {
		auto seg = input.get_segment(edge);
		ss << "[" << edge.id <<  "](" << seg.source() << " -> " << seg.target() << ") | ";
	}
	LOG(INFO) << ss.str();
}


/*** true if a lies above b relative to monotonicity line */
bool Data::isAbove(const Point& a, const Point &b) const {
	bool aAbove = monotonicityLine.has_on_positive_side(a);
	bool bAbove = monotonicityLine.has_on_positive_side(b);
	auto distA = CGAL::squared_distance(monotonicityLine,a);
	auto distB = CGAL::squared_distance(monotonicityLine,b);
	return ( aAbove && !bAbove) ||
		   ( aAbove &&  bAbove && distA > distB) ||
		   (!aAbove && !bAbove && distA < distB);
}


/*
 * NOTE: this only works if writeOBJ was invoced before, as we only
 * add a face with the indices that require the vertices already in
 * the file
 * */
void Data::addPolyToOBJ(const Config& cfg) const {
	double xt, yt, zt, xm, ym, zm;
	getNormalizer(*bbox,xt,xm,yt,ym,zt,zm);

	std::ofstream outfile (cfg.outputFileName,std::ofstream::binary | std::ofstream::app);

	outfile << "f";
	for(auto e : getPolygon()) {
		outfile << " " << e.u+1;
	}

	outfile << std::endl;
	outfile.close();
}
