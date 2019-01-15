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
	os << mv.vector << "," << mv.id;
	return os;
}

void Data::initialize(const Config& cfg) {
	/* load input vertices/polygon and weights */
	if(!loadFile(cfg.fileName)) { assert(false); return;}
}

Edge Data::getEdge(const uint& idx) const {
	assert(idx < (uint)polygon.size());
	return Edge( eA(idx), eB(idx) );
}

Edge Data::getEdge(const EdgeIterator& it) const {
	return Edge( eA((*it)[0]), eB((*it)[1]) );
}

bool Data::isEdgeCollinear(const uint& i, const uint& j) const {
	return CGAL::collinear(eA(i),eB(i),eA(j)) &&
		   CGAL::collinear(eA(i),eB(i),eB(j));
}

bool Data::loadFile(const std::string& fileName) {
	if(fileExists(fileName)) {
		std::ifstream in(fileName);

		std::string extension = (fileName.find(".")  != std::string::npos) ? fileName.substr(fileName.find_last_of(".")+1) : "";
		transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

		if( extension == "gml" ) {
			std::cout << "parsing gml" << std::endl;
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

	return !inputVertices.empty() && !polygon.empty();
}

bool Data::parsePOLY(const std::vector<std::string>& lines) {
	std::cout << "POLY: no yet supported!" << lines.size() << std::endl;
	return false;
}


bool Data::parseGML(std::istream &istream) {
	gml = GMLGraph::create_from_graphml(istream);
	basicInput = BasicInput();
	basicInput.add_graph(gml);

	InputPoints 	points;
	Polygon			poly;
	InputWeights 	weights;

	for(auto v : basicInput.vertices()) {
		points.push_back(v.p);
	}

	/* iterate over edges to construct polygon */
	auto edges = basicInput.edges();

	std::vector<uint> vertexEdgeMap(edges.size());
	for(uint i=0; i < edges.size(); ++i) {
		auto edge = edges[i];
		std::cout << " " << i << "," << edge.u; fflush(stdout);
		vertexEdgeMap[edge.u] = i;
	}

	std::cout << std::endl;

	auto eIdxStart = 0;
	auto eIt = eIdxStart;
	do {
		auto e = edges[eIt];
		std::cout << " " << e.u << "," << e.v; fflush(stdout);
		poly.push_back({{e.u-1, e.v-1}});
		weights.push_back(e.weight);
		eIt = vertexEdgeMap[e.v];
	} while(eIt != eIdxStart);

	uint lastEdgeA = poly.back()[1];
	uint lastEdgeB = poly.front()[0];

	if(!basicInput.has_edge(lastEdgeB,lastEdgeA)) {
		poly.push_back({{lastEdgeA, lastEdgeB }});
		weights.push_back(CORE_ONE);
		LOG(INFO) << "last edge missing! weight set to 1. ";
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

	std::vector<MonotoneVector> intervals;
	uint idCnt = 0;

	/* iterate over polygon and obtaining the 'monotonicity angle'
	 * if it exists. */
	do {
		edgeA = edgeB;
		edgeB = *edgeIt;
		vA = vB;
		vB = getEdge(edgeIt).to_vector();
		corner = v(edgeA[1]);

		/* ensure the vertex is reflex */
		if(CGAL::right_turn(corner-vA,corner,corner+vB)) {
			MonotoneVector a(vA,MonotoneType::END,idCnt);
			MonotoneVector b(vB,MonotoneType::START,  idCnt);
			intervals.push_back(a);
			intervals.push_back(b);

			a = MonotoneVector(Vector(-vA.x(),-vA.y()),MonotoneType::END,edgeA[1]);
			b = MonotoneVector(Vector(-vB.x(),-vB.y()),MonotoneType::START,  edgeA[1]);
			intervals.push_back(a);
			intervals.push_back(b);

			++idCnt;
		}
	} while(++edgeIt != polygon.end());

	if(intervals.empty()) {
		/* polygon is convex, let us choose the x-axis */
		monotonicityLine = Line(ORIGIN, ORIGIN + Vector(1,0));
		return true;
	}

	/* sort all vectors in interval in CCW order */
	std::sort(intervals.begin(),intervals.end(),MonVectCmp());

	/* iterate to first START vector */
	auto itStart = intervals.begin();
	for(;itStart->type != MonotoneType::START; ++itStart);

	uint activeCnt  = 0;		/* keep track how many intervals are active */
	auto it 		= itStart;
	bool success 	= false;
	do {
		if(it->type == MonotoneType::START) {
			++activeCnt;
		} else { /* type is MonotoneType::END */
			--activeCnt;
		}

		/* we found a window*/
		if(activeCnt == 0) {
			success = true;
		}

		++it;
		if(it == intervals.end()) {it = intervals.begin();}
	} while(it != itStart && !success);


	/* construct the monotonicity line */
	if(success) {
		Vector a = it->vector;
		it = (it == intervals.begin()) ? intervals.end()-1 : it-1;

		Vector b = it->vector;
		Vector c = a+b;
		monotonicityLine = Line(ORIGIN, ORIGIN + c.perpendicular(CGAL::POSITIVE));
		return true;
	} else { /* polygon is not monotone */
		return false;
	}
}
bool Data::monotoneSmaller(const Point& a, const Point& b) const {
	return (monotonicityLine.projection(b) - monotonicityLine.projection(a)).direction() == monotonicityLine.direction();
}
BBox Data::computeBoundingBox() const {
	auto box = BBox();
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

	return box;
}

Edge Data::confineRayToBBox(const Ray& ray) const {
	auto p = ray.source();

	for(auto e : {bbox.top, bbox.bottom, bbox.left, bbox.right}) {
		auto intersection = intersectElements<Ray,Edge>(ray,e);
		if( intersection != INFPOINT ) {
			return Edge(p,intersection);
		}
	}
	return Edge(p,INFPOINT);
}



void Data::printInput() const {
	std::cout << "Input Vertices: " << std::endl;
	for(auto point : inputVertices) {
		std::cout << "(" << point.x() << "," << point.y() << ") ";
	}
	std::cout << std::endl << "Input Polygon: " << std::endl;
	for(auto edge : polygon) {
		std::cout  << "(" << edge[0] << "," << edge[1] << ") ";
	}
	std::cout << std::endl;
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
