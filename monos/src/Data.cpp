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

void Data::initialize(const Config& cfg) {
	/* load input vertices/polygon and weights */
	if(cfg.isValid()) {
		if(cfg.use_stdin) {
			if(!parseGML(std::cin)) {exit(1);}
		} else if (!loadFile(cfg.fileName)) {
			exit(1);
		}
	}
}

Edge Data::getEdge(const uint& idx) const {
	assert(idx < (uint)polygon.size());
	return Edge( eA(idx), eB(idx) );
}

Edge Data::getEdge(const EdgeIterator& it) const {
	return getEdge(it - polygon.begin());
}

bool Data::isEdgeCollinear(const Edge& eA, const Edge& eB) const {
	return CGAL::parallel(eA,eB) && CGAL::collinear(eA.point(0),eA.point(1),eB.point(0));
}

bool Data::isEdgeCollinear(const uint& i, const uint& j) const {
	auto eA = getEdge(i);
	auto eB = getEdge(j);
	return isEdgeCollinear(eA,eB);
}

bool Data::loadFile(const std::string& fileName) {
	std::ifstream in;
	if(fileExists(fileName)) {
		in.open(fileName);
		std::string extension = (fileName.find(".")  != std::string::npos) ? fileName.substr(fileName.find_last_of(".")+1) : "";
		transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

		if( extension == "gml" || extension == "graphml" ) {
			LOG(INFO) << "parsing gml" << std::endl;
			if(!parseGML(in)) {return false;}
		} else {
			std::vector<std::string> lines;
			for(std::string line; getline(in, line);) {
				lines.push_back(line);
			}

			if(extension == "obj") {
				/* wavefront obj format - https://en.wikipedia.org/wiki/Wavefront_.obj_file */
				if(!parseOBJ(lines)) {return false;}
			} else if(extension == "poly") {
				/* triangle's poly file format - https://www.cs.cmu.edu/~quake/triangle.poly.html */
				if(!parsePOLY(lines)) {return false;}
			}
		}
		return true;
	}

	return false;
}


bool Data::parseOBJ(const std::vector<std::string>& lines) {
	std::vector<std::vector<uint>> faces;

	InputPoints 	points;
	Polygon			poly;
	InputWeights 	weights;

	for(auto l : lines) {
		std::istringstream buf(l);
		std::istream_iterator<std::string> beg(buf), end;
		std::vector<std::string> tokens(beg, end);

		auto isVertex  = false;
		auto isFacet   = false;

		bool setX=true, setY=true, setZ=true;
		double p_x=-INFINITY, p_y=-INFINITY, p_z=-INFINITY;

		std::vector<uint> facetList;

		for(auto& s : tokens) {
			/* comments, and not supported or needed obj parameters */
			if(s == "#" || s == "o" || s == "vt" || s == "vn" ||
			   s == "g" || s == "usemtl" || s == "s" || s == "off" ||
			   s == "mtllib" || s == "Ka" || s == "Kd" || s == "Ks" ||
			   s == "d" || s == "newmtl" || s == "illum" || *s.begin() == 'm' ||
			   *s.begin() == '-' || *s.begin() == 'b' || s == "l") {
				break;
			}

			if(isVertex) {
				if(setX) {
					p_x = atof(s.c_str());
					setX = false;
				} else if(setY){
					p_y = atof(s.c_str());
					setY = false;
				} else if(setZ) {
					p_z = atof(s.c_str());
					setZ = false;
				}
			} else if(isFacet) {
				facetList.push_back(std::atoi(s.c_str()));
			}

			if(s == "v") {isVertex  = true;}
			if(s == "f") {isFacet   = true;}
		}

		if(!setX && !setY) {
			points.push_back(Point(p_x,p_y));
		}

		if(!facetList.empty()) {
			faces.push_back(facetList);
		}
	}

	if(!faces.empty()) {
		auto face = faces[0];
		for(uint i=0; i < face.size(); ++i) {
			poly.push_back( {{face[i]+OBJOFFSET,face[(i+1)%face.size()]+OBJOFFSET }} );
			/* input weights are realized via GraphML -- .gml files*/
			weights.push_back(1.0);
		}
	} else {
		LOG(WARNING) << "More than one face!";
	}

	/* initialize const input variables */
	inputVertices 	= points;
	polygon 		= poly;
	edgeWeights 	= weights;

	/* construct BasicInput for GUI */
	if(gui)  {
		basicInput.add_list(inputVertices,polygon);
	}
	return !inputVertices.empty() && !polygon.empty();
}

bool Data::parsePOLY(const std::vector<std::string>& lines) {
	LOG(INFO) << "POLY: no yet supported!" << lines.size();
	return false;
}


bool Data::parseGML(std::istream &istream) {
	gml = GMLGraph::create_from_graphml(istream);
	basicInput = BasicInput();
	basicInput.add_graph(gml);

	InputPoints 	points;

	for(auto v : basicInput.vertices()) {
		points.push_back(v.p);
	}

	/* iterate over edges to construct polygon */
	auto edges = basicInput.edges();

	Polygon			poly(edges.size());
	InputWeights 	weights(edges.size());

	for(uint i=0; i < edges.size(); ++i) {
		auto edge = edges[i];
		auto min = std::min(edge.u,edge.v);
		auto max = std::max(edge.u,edge.v);
		assert(min != max);

		if(min != 0 || (min == 0 && max == 1) ) {
			poly[min] = {{edge.u, edge.v}};
			weights[min] = edge.weight;
		} else {
			poly[max] = {{edge.v, edge.u}};
			weights[max] = edge.weight;
		}
	}

	/* initialize const input variables */
	inputVertices 	= points;
	polygon 		= poly;
	edgeWeights 	= weights;

	return !inputVertices.empty() && !polygon.empty();
}

bool Data::ensureMonotonicity() {
	assert(polygon.size() > 2);

	auto edgeIt = polygon.begin();
	IndexEdge edgeA, edgeB;
	Vector vA, vB, intervalA, intervalB;
	Point corner;

	edgeB  	= *edgeIt;
	edgeA  	= edgeB;
	vB 		= getEdge(edgeIt).to_vector();

	++edgeIt;

	intervalA = vA;
	intervalB = vA;

	std::set<MonotoneVector,MonVectCmp> intervals;
	uint idCnt = 0;

	/* iterate over polygon and obtaining the 'monotonicity angle'
	 * if it exists. */
	do {
		edgeA = edgeB;
		edgeB = *edgeIt;
		vA = vB;
		vB = getEdge(edgeIt).to_vector();
		corner = v(edgeA[1]);
		LOG(INFO) << ": " << vB;
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
	} while(++edgeIt != polygon.end());

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

	return true;
}

/* test if given line is a line where the input polygon is monotone to */
bool Data::testMonotonicityLineOnPolygon(const Line line) const {
	uint startIdx = 0;
	Point pStart  = eA(startIdx);
	for(uint i = startIdx+1; i < polygon.size(); ++i) {
		auto p = eA(i);
		if(monotoneSmaller(line,p,pStart)) {
			startIdx = i; pStart = p;
		}
	}

	/* startIdx is 'leftmost' edge, start walking 'rightwards' until we violate the monotonicity */
	bool monotone  = true, rightward = true;
	auto dir = line.direction().perpendicular(CGAL::POSITIVE);
	LOG(INFO) << "test dir: " << dir;
	auto idxIt = startIdx;
	do {
		LOG(INFO) << "pa " << eA(idxIt) << ", pb " << eB(idxIt);
		auto testLine = Line(eA(idxIt),-dir);
		if(rightward && testLine.has_on_negative_side(eB(idxIt))) {
			LOG(INFO) << "end rightward";
			rightward = false;
		} else if(!rightward && testLine.has_on_positive_side(eB(idxIt))) {
			LOG(INFO) << "end monotone";
			monotone = false;
		}

		if(++idxIt >= polygon.size()) {idxIt = 0;}
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
	return perpL.has_on_positive_side(a);
}
bool Data::rayPointsLeft(const Ray& ray) const {
	Point Pa = monotonicityLine.point(0);
	Point Pb = Pa + ray.to_vector();
	return monotoneSmaller(Pb,Pa);
}

BBox Data::computeBoundingBox() const {
	auto box = BBox(0,0,0,0,v(0).x(),v(0).x(),v(0).y(),v(0).y(),0,0,v(0),v(0));
	for(uint i=0; i < inputVertices.size(); ++i) {
		if(i!= box.xMinIdx && v(i).x() < v(box.xMinIdx).x()) {box.xMinIdx = i;}
		if(i!= box.xMaxIdx && v(i).x() > v(box.xMaxIdx).x()) {box.xMaxIdx = i;}
		if(i!= box.yMinIdx && v(i).y() < v(box.yMinIdx).y()) {box.yMinIdx = i;}
		if(i!= box.yMaxIdx && v(i).y() > v(box.yMaxIdx).y()) {box.yMaxIdx = i;}

		if(i != box.monotoneMinIdx && monotoneSmaller(v(i),v(box.monotoneMinIdx))) {
			box.monotoneMinIdx = i;
		}
		if(i != box.monotoneMaxIdx && monotoneSmaller(v(box.monotoneMaxIdx),v(i))) {
			box.monotoneMaxIdx = i;
		}
	}

	box.xMin = v(box.xMinIdx).x();
	box.xMax = v(box.xMaxIdx).x();
	box.yMin = v(box.yMinIdx).y();
	box.yMax = v(box.yMaxIdx).y();

	box.top    = Edge(Point(box.xMax,box.yMax),Point(box.xMin,box.yMax));
	box.bottom = Edge(Point(box.xMin,box.yMin),Point(box.xMax,box.yMin));
	box.left   = Edge(Point(box.xMin,box.yMax),Point(box.xMin,box.yMin));
	box.right  = Edge(Point(box.xMax,box.yMin),Point(box.xMax,box.yMax));

	box.monotoneMin = v(box.monotoneMinIdx);
	box.monotoneMax = v(box.monotoneMaxIdx);

	LOG(INFO) << box;

	return box;
}

Edge Data::confineRayToBBox(const Ray& ray) const {
	Point iA(ray.source()), iB(INFPOINT);

	for(auto e : {bbox.top, bbox.bottom, bbox.left, bbox.right}) {
		auto intersection = intersectElements<Ray,Edge>(ray,e);
		if( intersection != INFPOINT && intersection != iA) {
			iB = intersection;
		}
	}
	return Edge(iA,iB);
}



void Data::printInput() const {
	std::stringstream ss;
	ss << "Input Vertices: " << std::endl;
	for(auto point : inputVertices) {
		ss << "(" << point.x() << "," << point.y() << ") ";
	}
	ss << std::endl << "Input Polygon: " << std::endl;
	uint cnt = 0;
	for(auto edge : polygon) {
		ss << "(" << edge[0] << "," << edge[1] << ")N[" << getEdge(cnt++).to_vector() << "] ";
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
	if(cfg.outputType == OutputType::OBJ) {
		double xt, yt, zt, xm, ym, zm;
		getNormalizer(bbox,xt,xm,yt,ym,zt,zm);

		std::ofstream outfile (cfg.outputFileName,std::ofstream::binary | std::ofstream::app);

		outfile << "f";
		for(auto e : polygon) {
			outfile << " " << e[0]+1;
		}

		outfile << std::endl;
		outfile.close();
	}
}
