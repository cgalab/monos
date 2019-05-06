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
    case ArcType::RAY : os << " ray"; break;
    case ArcType::NORMAL : os << " edge"; break;
    }
    return os;
}


Exact normalDistance(const Line& l, const Point& p) {
	return CGAL::squared_distance(l,p);
}

bool do_intersect(const Ray& ray, const Arc& arc) {
	switch(arc.type) {
	case ArcType::NORMAL: return CGAL::do_intersect(ray,arc.edge);
	case ArcType::RAY: return CGAL::do_intersect(ray,arc.ray);

	case ArcType::DISABLED:
	default: LOG(WARNING) << "Not supposed to happen!"; return false;
	}
}

Point intersectRayArc(const Ray& ray, const Arc& arc) {
	return (arc.type == ArcType::NORMAL) ? intersectElements(ray,arc.edge) : intersectElements(ray,arc.ray);
}

//Point intersectEdgeEdge(Edge *a, Edge *b) {
//	Point intersectionPoint = INFPOINT;
//
//	auto result = CGAL::intersection(*a, *b);
//	if (result) {
//		if (const Edge* s = boost::get<Edge>(&*result)) {
//			LOG(WARNING) << "Intersection forms a segment";
//			std::cout << *s << std::endl;
//			return intersectionPoint;
//		} else {
//			const Point* p = boost::get<Point>(&*result);
//			return Point(*p);
//		}
//	}
//
//	return intersectionPoint;
//}
//
//Point intersectRayEdge(Ray *r, Edge *e) {
//	Point intersectionPoint = INFPOINT;
//
//	auto result = CGAL::intersection(*r, *e);
//	if (result) {
//		if (const Edge* s = boost::get<Edge>(&*result)) {
//			LOG(WARNING) << "Intersection forms a segment";
//			std::cout << *s << std::endl;
//			return intersectionPoint;
//		} else {
//			const Point* p = boost::get<Point>(&*result);
//			return Point(*p);
//		}
//	}
//
//	return intersectionPoint;
//}
//
//Point intersectBisectors(Ray *a, Ray *b) {
//	Point intersectionPoint = INFPOINT;
//
//	if(CGAL::parallel(*a, *b)) {
//		LOG(INFO) << "Rays parallel (not intersecting)";
//		return intersectionPoint;
//	}
//	auto result = CGAL::intersection(*a, *b);
//	if (result) {
//		if (const Edge* s = boost::get<Edge>(&*result)) {
//			LOG(WARNING) << "Intersection forms a segment";
//			std::cout << *s << std::endl;
//			return intersectionPoint;
//		} else {
//			const Point* p = boost::get<Point>(&*result);
//			return Point(*p);
//		}
//	}
//
//	return intersectionPoint;
//}
//
//Point intersectLines(Line *a, Line *b) {
//	Point intersectionPoint = INFPOINT;
//
//	//assert(!isLinesParallel(a, b));
//
//	auto result = CGAL::intersection(*a, *b);
//	if (result) {
//		if (const Edge* s = boost::get<Edge>(&*result)) {
//			LOG(WARNING) << "Intersection forms a segment";
//			std::cout << *s << std::endl;
//			return intersectionPoint;
//		} else {
//			const Point* p = boost::get<Point>(&*result);
//			return Point(*p);
//		}
//	}
//
//	LOG(WARNING) << "No LINE/LINE intersecting?!";
//	return intersectionPoint;
//}

