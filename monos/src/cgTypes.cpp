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

std::ostream& operator<< (std::ostream& os, const BBox& box) {
	os << "box l,r [" << box.xMinIdx << "," << box.xMaxIdx << "]";
	os << " b,t [" << box.yMinIdx << "," << box.yMaxIdx << "]";
	os << " m-min,m-max [" << box.monotoneMinIdx << "," << box.monotoneMaxIdx << "]";

	return os;
}

std::ostream& operator<< (std::ostream& os, const Bisector& bis) {
	if(bis.isRay()) {
		os << "r ";
	} else {
		os << "l ";
	}
	os << bis.point(0) << " dir: " << bis.direction();
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
	os << "time: " << node.time << ", point: " << node.point;
	os << std::boolalpha << " g: " << node.ghost;
    return os;
}

std::ostream& operator<< (std::ostream& os, const Event& event) {
    os << "(" << event.eventTime.doubleValue() << " : " << event.eventPoint << ")["
    		<< event.edges[0] << ","
			<< event.edges[1] << ","
			<< event.edges[2] << "]";
    auto it1 = ChainRef(event.chainEdge);
    os << " it(" << *(--it1) << "," << *(++it1) << "," << *(++it1) << ")";
    return os;
}

std::ostream& operator<< (std::ostream& os, const Arc& arc) {
    os << "N(" << arc.firstNodeIdx << "," << arc.secondNodeIdx << ")"
    		<< " E(" << arc.leftEdgeIdx << "," << arc.rightEdgeIdx << ")";
    switch(arc.type) {
    case ArcType::DISABLED : os << " disabled"; break;
    case ArcType::RAY : os << " ray: " << arc.ray; break;
    case ArcType::NORMAL : os << " edge: " << arc.edge; break;
    }
    return os;
}


Exact normalDistance(const Line& l, const Point& p) {
	return CGAL::squared_distance(l,p);
}

bool do_intersect(const Bisector& bis, const Edge& edge) {
	return (bis.isRay()) ? CGAL::do_intersect(bis.ray,edge) : CGAL::do_intersect(bis.line,edge);
}

bool do_intersect(const Bisector& bis, const Arc& arc) {
	switch(arc.type) {
	case ArcType::NORMAL: 	if(bis.isRay()) {return CGAL::do_intersect(bis.ray,arc.edge);}
							else {return CGAL::do_intersect(bis.line,arc.edge);}
	case ArcType::RAY: 		if(bis.isRay()) {return CGAL::do_intersect(bis.ray,arc.ray);}
							else {return CGAL::do_intersect(bis.line,arc.ray);}

	case ArcType::DISABLED:
	default: LOG(WARNING) << "Not supposed to happen!"; return false;
	}
}

Point intersectBisectorArc(const Bisector& bis, const Arc& arc) {
	auto lRef = bis.supporting_line();

	bool bisAA = lRef.is_horizontal() || lRef.is_vertical();

	if(arc.isAA() || bisAA) {
		LOG(WARNING) << "AA elements might cause problems!";

		if(arc.isEdge() && !bisAA) {
			if(lRef.has_on_positive_side(arc.point(0)) && lRef.has_on_positive_side(arc.point(1))) {
				return INFPOINT;
			}
			if(lRef.has_on_negative_side(arc.point(0)) && lRef.has_on_negative_side(arc.point(1))) {
				return INFPOINT;
			}
		}

		Point P = INFPOINT;

		if(arc.isEdge()) {
			P = intersectElements(bis.supporting_line(),arc.edge);
			return P;
		} else {
			P = intersectElements(bis.supporting_line(),arc.supporting_line());
		}

		if(P != INFPOINT) {
			auto arcSup = arc.supporting_line();
			auto arcNormalLineA = arcSup.perpendicular(arc.point(0));
			if(arc.isEdge()) {
				auto arcNormalLineB = arcSup.perpendicular(arc.point(1));

				if(P != arc.point(0) && P != arc.point(1) && (arcNormalLineB.has_on_negative_side(P) || arcNormalLineA.has_on_positive_side(P)) )  {
					return INFPOINT;
				}
			} else {
				if(P != arc.point(0) && arcNormalLineA.has_on_positive_side(P))  {
					return INFPOINT;
				}
			}
		}

		return P;

	} else if(arc.isEdge()) {
		if(bis.isRay()) {
			return intersectElements(bis.ray,arc.edge);
		} else {
			return intersectElements(bis.line,arc.edge);
		}

//		Point a = arc.edge.point(0);
//		Point b = arc.edge.point(1);
//
//		if( (lRef.has_on_positive_side(a) && lRef.has_on_positive_side(b)) ||
//				(lRef.has_on_negative_side(a) && lRef.has_on_negative_side(b))
//		) {
//			return INFPOINT;
//		} else {
//			if(bis.isRay()) {
//				return intersectElements(bis.ray,arc.edge);
//			} else {
//				return intersectElements(bis.line,arc.edge);
//			}
//		}
	} else {
		if(bis.isRay()) {
			return intersectElements(bis.ray,arc.ray);
		} else {
			return intersectElements(bis.line,arc.ray);
		}
	}
}

Point intersectBisectorEdge(const Bisector& bis, const Edge& edge) {
	if(bis.isRay()) {
		return intersectElements(bis.ray,edge);
	} else {
		return intersectElements(bis.line,edge);
	}
}

Point intersectArcArc(const Arc& arcA, const Arc& arcB) {
	if(arcA.isEdge()) {
		if(arcB.isEdge()) {
			return intersectElements(arcB.edge,arcA.edge);
		} else {
			return intersectElements(arcB.ray,arcA.edge);
		}
	} else {
		if(arcB.isEdge()) {
			return intersectElements(arcB.edge,arcA.ray);
		} else {
			return intersectElements(arcB.ray,arcA.ray);
		}
	}
}

Point intersectRayArc(const Ray& ray, const Arc& arc) {
	return (arc.type == ArcType::NORMAL) ? intersectElements(ray,arc.edge) : intersectElements(ray,arc.ray);
}


