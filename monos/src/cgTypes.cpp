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

#include "cgTypes.h"

std::ostream& operator<< (std::ostream& os, const TimeEdge& te) {
	os << te.edgeIdx << " at time " << te.time;
	return os;
}

std::ostream& operator<< (std::ostream& os, const Vertex& vertex) {
	os << "v " << vertex.id << ": " << vertex.p;
	return os;
}
std::ostream& operator<< (std::ostream& os, const Edge& edge) {
	os << "e " << edge.id << ": " << edge.u << " -> " << edge.v;
	return os;
}

std::ostream& operator<< (std::ostream& os, const BBox& box) {
	os << "bbox monMin/monMax: " << box.monMin << " | " << box.monMax;
	return os;
}

std::ostream& operator<< (std::ostream& os, const Node& node) {
	if(node.isTerminal()) {
		os << "t ";
	} else if(node.type == NodeType::NORMAL){
		os << "n ";
	} else {
		os << "d ";
	}
	os << "time: " << node.time.doubleValue() << ", point: " << node.point.x().doubleValue() << "," << node.point.y().doubleValue();
	os << ", arcs " << node.arcs.size() << ": ";
	for(auto a : node.arcs) {
		os << a << " ";
	}
    return os;
}

std::ostream& operator<< (std::ostream& os, const Event& event) {
    os << "(" << event.eventTime.doubleValue() << " : " << event.eventPoint.x().doubleValue() << "," << event.eventPoint.y().doubleValue() << ")["
    		<< event.leftEdge << ","
			<< event.mainEdge << ","
			<< event.rightEdge << "]";
    auto it1 = ChainRef(event.chainEdge);
    os << " it(" << *(--it1) << "," << *(++it1) << "," << *(++it1) << ")";
    return os;
}

std::ostream& operator<< (std::ostream& os, const Arc& arc) {
	os << "id " << arc.id << " ";
	if(arc.firstNodeIdx == MAX) {
		os << "N(MAX,";
	} else {
		os << "N(" << arc.firstNodeIdx << ",";
	}
	if(arc.secondNodeIdx == MAX) {
		os << "MAX)";
	} else {
		os << arc.secondNodeIdx << ")";
	}
    os << " E(" << arc.leftEdgeIdx << "," << arc.rightEdgeIdx << ")";
    switch(arc.type) {
    case ArcType::DISABLED : os << " disabled"; break;
    case ArcType::RAY : os << " ray"; break; 		//: " << arc.ray; break;
    case ArcType::NORMAL : os << " edge"; break; 	//: " << arc.edge; break;
    }
    if(arc.is_vertical()) {
    	os << " -v- ";
    } else if(arc.is_horizontal()) {
    	os << " -h- ";
    }
    return os;
}


NT normalDistance(const Line& l, const Point& p) {
	return CGAL::squared_distance(l,p);
}


//ul getArcsCommonNodeIdx(const Arc& arcA, const Arc& arcB) {
//	for(auto i : {arcA.firstNodeIdx,arcA.secondNodeIdx}) {
//		for(auto j : {arcB.firstNodeIdx,arcB.secondNodeIdx}) {
//			if(i != MAX && j != MAX && i == j) {
//				return i;
//			}
//		}
//	}
//	return MAX;
//}

void getNormalizer(const BBox& bbox, double& xt, double& xm, double& yt, double& ym, double& zt, double& zm) {
	double x_span  = (1.0/OBJSCALE) * (bbox.xMax.p.x().doubleValue() - bbox.xMin.p.x().doubleValue());
	double y_span  = (1.0/OBJSCALE) * (bbox.yMax.p.y().doubleValue() - bbox.yMin.p.y().doubleValue());

	xt = bbox.xMin.p.x().doubleValue() + (0.5 * (OBJSCALE) * x_span);
	yt = bbox.yMin.p.y().doubleValue() + (0.5 * (OBJSCALE) * y_span);
	zt = 0.0;

	xm = (x_span + smallEPS > 0.0) ? OBJSCALE/x_span : 1;
	ym = (y_span + smallEPS > 0.0) ? OBJSCALE/y_span : 1;

	xm = ym = std::max(xm,ym);
	zm = xm;
}
