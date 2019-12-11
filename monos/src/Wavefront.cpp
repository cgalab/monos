/* monos is written in C++.  It computes the weighted straight skeleton
 * of a monotone polygon in asymptotic n log n time and linear space.
 *
 * Copyright 2018, 2019 Günther Eder - geder@cs.sbg.ac.at
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
#include "Wavefront.h"

void Wavefront::InitializeEventsAndPathsPerEdge() {
	/* set up empty events for every edge;
	* set up initial target node for pathfinder
	**/
	events.resize(data.getPolygon().size(),Event());
	for(const auto& e : data.getPolygon()) {
		pathFinder.emplace_back(EndNodes(e.u, e.v));
	}
	for(unsigned long i = 0; i < data.getPolygon().size(); ++i) {
		events[i].mainEdge = i;
	}
}

void Wavefront::InitializeNodes() {
	/* create all terminal nodes of skeleton (vertices of input) */
	for(const auto& v : data.getVertices()) {
		addNode(v.p, 0, NodeType::TERMINAL);
	}
}


bool Wavefront::ComputeSkeleton(ChainType type) {
	auto& chain     = getChain(type);
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {return true;}

	/************************************/
	/* 	filling the priority queue 		*/
	/************************************/
	if(!InitSkeletonQueue(chain)) {return false;}


	/*********************************************/
	/* compute skeleton by working through queue */
	/*********************************************/
	LOG(INFO) << "PRINT CHAIN BEFORE SingleDequeues";
	printChain(chain);

	currentTime = 0;

	while(SingleDequeue(chain));

	/* DEBUG: if needed print chain */
	//LOG(INFO) << "PRINT CHAIN BEFORE FINISHING";
	//	printChain(chain);

	/***********************************************************************/
	/* construct rays from remaining edges in chain, i.e,. unbounded faces */
	/***********************************************************************/
	if(!FinishSkeleton(chain)) {return false;}

	nextState();

	return true;
}

bool Wavefront::InitSkeletonQueue(Chain& chain) {
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {return true;}

	auto chainIterator = chain.begin();
	/* first edge defines an unbounded face in the skeleton induced graph */
	ul aEdgeIdx = *chainIterator;
	++chainIterator;
	ul bEdgeIdx = *chainIterator;
	ChainRef it = ChainRef(chainIterator);
	++chainIterator;
	ul cEdgeIdx = *chainIterator;

	/************************************/
	/* 	filling the priority queue 		*/
	/************************************/
	do {
		cEdgeIdx = *chainIterator;
		/* create Event and add it to the queue */
		auto event = getEdgeEvent(aEdgeIdx,bEdgeIdx,cEdgeIdx,it);

		events[event.mainEdge] = event;

		/* iterate over the chainIterator */
		it = chainIterator;
		++chainIterator;
		aEdgeIdx = bEdgeIdx;
		bEdgeIdx = cEdgeIdx;
	} while(chainIterator != chain.end());

	LOG(INFO) << "number of events " << events.size();
	eventTimes = new EventQueue(&events, chain);

	currentTime = 0;

	return true;
}

bool Wavefront::SingleDequeue(Chain& chain) {
	LOG(INFO) << std::endl << "########################### SingleDequeue ("<< eventTimes->size() << ") ##############################";

	if(!eventTimes->empty()) {

		const Event* e = eventTimes->peak()->priority.e;
		ul edgeIdx = e->mainEdge;
		eventTimes->drop_by_tidx(edgeIdx);

		if(currentTime <= e->eventTime && e->isEvent()) {
			currentTime = e->eventTime;

			if(eventTimes->empty() || eventTimes->peak()->priority.e->eventTime != currentTime) {
				HandleSingleEdgeEvent(chain,e);
			} else {
				std::vector<const Event*> eventList = {e};

				while(!eventTimes->empty() && eventTimes->peak()->priority.e->eventTime == currentTime) {
					e = eventTimes->peak()->priority.e;
					if(e->isEvent()) {
						eventList.emplace_back(e);
					}
					eventTimes->drop_by_tidx(e->mainEdge);
				}
				HandleMultiEvent(chain,eventList);
			}

		} else if(e->eventTime == MAX) {
			/* the remaining events in the queue have MAX time, thus, we are done! */
			return false;
		}

		return true;
	}

	/* event queue is emtpy, we are done! */
	return false;
}



/* due to the monotonicity we only have two scenarios:
 * (i) these events amounts to exactly one point with multiple collapsing edges:
 * --> then we continue with the bisector between the first and last involved
 *     edge.
 * (ii)  otherwise a vertical bisector line is involved, we have
 *       exactly two points:
 * --> after adding both points (connected with a vertical segment) we apply
 *     (i) for the 'upper' (or higher) point. */
void Wavefront::HandleMultiEvent(Chain& chain, std::vector<const Event*> eventList) {
	if(eventList.size() == 1) {
		LOG(INFO) << "size 1 ";
		HandleSingleEdgeEvent(chain,eventList[0]);
	} else {
		std::set<Point> points;
		std::set<int,std::less<int> > eventEdges;
		LOG(INFO) << "events:";
		for(const auto& e : eventList) {
			if(e->mainEdge == *(e->chainEdge))  {
				LOG(INFO) << "event is still true!";
			}

			points.insert(e->eventPoint);
			eventEdges.insert(e->leftEdge);
			eventEdges.insert(e->mainEdge);
			eventEdges.insert(e->rightEdge);
		}

		if(points.size() > 1) {
			/* scenario (ii) : ONLY TWO POINTS SHOULD BE POSSIBLE!
			 * this scenario implies a vertical segment, i.e., perpendicular to the
			 * monotonicity line. This means only two event points can occur on this
			 * line, otherwise the polygon can not be monotone */
			assert(points.size() < 3);
			LOG(INFO) << "scenario (ii)";
			/* ensure A is the lower point in resp. to monot. line */
			auto it = points.begin();
			Point A = *it; ++it;
			Point B = *it;

			for(;it!=points.end();++it) {
				B = *it;
				bool aAboveB = data.isAbove(A,B);
				if( (  aAboveB  &&  isLowerChain(chain)) ||
					( !aAboveB  && !isLowerChain(chain))) {
					std::swap(A,B);
				}
			}
			std::vector<const Event*> partEventListA, partEventListB;
			for(auto e : eventList) {
				if(e->eventPoint == A) {
					partEventListA.emplace_back(e);
				} else {
					partEventListB.emplace_back(e);
				}
			}

			assert(partEventListA.size() >= 1);

			/* handling the 'lower' eventpoint changes the other event, thus
			 * we only handle this lower event */
			HandleSingleEdgeEvent(chain,partEventListA[0]);
		} else {
			/* scenario (i) */
			LOG(INFO) << "scenario (i)";
			HandleMultiEdgeEvent(chain,eventList);
		}
	}
}

void Wavefront::HandleMultiEdgeEvent(Chain& chain, std::vector<const Event*> eventList) {
	LOG(INFO) << "HandleMultiEdgeEvent! (one point, multiple edge collapses)";

	auto anEvent = eventList[0];
	auto nodeIdx = addNode(anEvent->eventPoint,anEvent->eventTime);
	auto& node = *getNode(nodeIdx);

	/* add the single node, all arcs connect to this node */
	LOG(INFO) << "adding node: " << node;

	std::set<ul> mainEdges;
	std::set<ul> leftEdges;
	std::set<ul> rightEdges;
	std::set<std::pair<ul,ul>> pairs;
	for(auto event : eventList) {
		leftEdges.insert(event->leftEdge);
		rightEdges.insert(event->rightEdge);

		std::vector<ul> va = {event->mainEdge,event->leftEdge },
				          vb = {event->mainEdge,event->rightEdge};

		for(auto v : { va, vb } ) {
			auto ta = std::pair<ul,ul>(v[0],v[1]);
			pairs.insert(ta);
		}
	}

	std::set<ul> doneNeighbour;

	for(auto p : pairs) {
		if(doneNeighbour.find(p.second) != doneNeighbour.end()) {continue;}

		auto paths = pathFinder[p.first];
		ul pIdx = MAX;
		if(leftEdges.find(p.second) != leftEdges.end()) {
			pIdx = paths.a;
		}
		if(rightEdges.find(p.second) != rightEdges.end()) {
			pIdx = paths.b;
		}
		assert(pIdx != MAX);
		addArc(pIdx,nodeIdx,p.first,p.second);

		doneNeighbour.insert(p.first);
	}

	for(auto event : eventList) {
		/* update path finder for left and right edge */
		pathFinder[event->leftEdge ].b = nodeIdx;
		pathFinder[event->mainEdge ].a = nodeIdx;
		pathFinder[event->mainEdge ].b = nodeIdx;
		pathFinder[event->rightEdge].a = nodeIdx;
	}

	auto chainIt = chain.begin();

	for(auto event : eventList) {
		/* remove this edges from the chain (wavefront) */
		chainIt = chain.erase(event->chainEdge);
		disableEdge(event->mainEdge);
	}

	if(chainIt != chain.begin()) {
		--chainIt;

		auto idxA = *(--chainIt);
		auto idxB = *(++chainIt);
		auto idxC = *(++chainIt);
		auto idxD = *(++chainIt);
		--chainIt;
		--chainIt;
		ChainRef it1 = ChainRef(chainIt);
		ChainRef it2 = ChainRef(++chainIt);

		if(idxA != idxB && idxB != idxC && idxC != idxD) {
			/**/
			LOG(INFO) << "TODO: only reference and add arcs if we are not before/after the chain ends!";

			/* only if in current chain! */
			pathFinder[idxB].b = nodeIdx;
			pathFinder[idxC].a = nodeIdx;

			auto e1 = getEdgeEvent(idxA,idxB,idxC,it1);
			auto e2 = getEdgeEvent(idxB,idxC,idxD,it2);

			LOG(INFO) << "indices A,B,C,D: " << idxA << " " << idxB << " " << idxC << " " << idxD;
			LOG(INFO) << e1;
			LOG(INFO) << e2;

			if(idxB != chain.front()) {
				updateInsertEvent(e1);
			}
			if(idxC != chain.back()) {
				updateInsertEvent(e2);
			}
		} else {
			LOG(INFO) << "EQUAL indices A,B,C,D: " << idxA << " " << idxB << " " << idxC << " " << idxD;
		}
	} else {
		LOG(INFO) << "chain is at beginning with size: " << chain.size();
	}
}

void Wavefront::HandleSingleEdgeEvent(Chain& chain,  const Event* event) {
	/* build skeleton from event */
	addNewNodefromEvent(*event);

	/* check neighbours for new events, and back into the queue */
	/* edges B,C are the two edges left, right of the event edge,
	 * A,D their respective neighbours */
	updateNeighborEdgeEvents(*event,chain);

	/* remove this edge from the chain (wavefront) */
	chain.erase(event->chainEdge);
}

bool Wavefront::FinishSkeleton(Chain& chain) {
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {return true;}
	Point pCheck;

	ul aEdgeIdx, bEdgeIdx;
	auto chainIterator = chain.begin();
	/***********************************************************************/
	/* construct rays from remaining edges in chain, i.e,. unbounded faces */
	/***********************************************************************/
	if(chain.size() > 1) {
		LOG(INFO) << " ------------------ FINISH SKELETON ------------------";
		chainIterator = chain.begin();
		aEdgeIdx = *chainIterator; ++chainIterator;
		do {
			bEdgeIdx = *chainIterator;
			LOG(INFO) << "chain edges: " <<  aEdgeIdx << ", " << bEdgeIdx;
			/* last node on path of both edges must be the same, get that node */
			auto endNodeIdx = pathFinder[aEdgeIdx].b;
			auto node = &nodes[endNodeIdx];

			const Line& la = data.get_line(aEdgeIdx);
			const Line& lb = data.get_line(bEdgeIdx);

			auto bisSimple = (!isCollinear(la,lb)) ? data.simpleBisector(la,lb) : getNormalBisector(aEdgeIdx,bEdgeIdx,la);

			pCheck = la.point(0) + bisSimple.to_vector();
			if(!la.has_on_positive_side(pCheck)) {bisSimple = bisSimple.opposite();}

			addArcRay(endNodeIdx,aEdgeIdx,bEdgeIdx,Ray(node->point,bisSimple.direction()));

			/* iterate over remaining chain */
			aEdgeIdx = bEdgeIdx;
		} while(++chainIterator != chain.end());
	}

	return true;
}

void Wavefront::updateNeighborEdgeEvents(const Event& event, const Chain& chain) {
	ul edgeA, edgeB, edgeC, edgeD;
	edgeB = event.leftEdge;
	edgeC = event.rightEdge;
	LOG(INFO) << event;
	ChainRef it(event.chainEdge);
	--it;

	if(*it == edgeB && edgeB != chain.front()) {
		--it;		edgeA = *it;

		it = ChainRef(event.chainEdge); --it;
		auto neighborEvent = getEdgeEvent(edgeA,edgeB,edgeC,it);
		updateInsertEvent(neighborEvent);
	}

	it = ChainRef(event.chainEdge);
	++it;
	if(*it == edgeC && edgeC != chain.back()) {
		++it;		edgeD = *it;

		it = ChainRef(event.chainEdge); ++it;
		auto neighborEvent = getEdgeEvent(edgeB,edgeC,edgeD,it);
		updateInsertEvent(neighborEvent);
	}
}

void Wavefront::updateInsertEvent(Event& event) {
	/* check if edge has already an event in the queue */

	if(event.eventTime < currentTime) {
		event.eventTime = MAX;
		event.eventPoint = INFPOINT;
		LOG(INFO) << "!! we set " << event.mainEdge << " to MAX!";
	}

	events[event.mainEdge] = event;
	eventTimes->update_by_tidx(event.mainEdge);
}

Event Wavefront::getEdgeEvent(const ul& aIdx, const ul& bIdx, const ul& cIdx, const ChainRef& it) const {
	const Line& a = data.get_line(aIdx);
	const Line& b = data.get_line(bIdx);
	const Line& c = data.get_line(cIdx);

	LOG(INFO) << "get event " << aIdx << "," << bIdx << "," << cIdx;

	/* compute bisector from edges */
	/* lets first test if this is too expexive */
	auto abBisL = (a != b) ? data.simpleBisector(a,b) : getNormalBisector(aIdx,bIdx,b);
	auto bcBisL = (b != c) ? data.simpleBisector(b,c) : getNormalBisector(bIdx,cIdx,b);

	LOG(INFO) << "AB is collinear: " << isCollinear(a,b) << " -- BC is collinear: " << isCollinear(b,c);;

	auto intersectionSimple = intersectElements(abBisL, bcBisL);
	if( intersectionSimple != INFPOINT && b.has_on_positive_side(intersectionSimple) ) {
		auto distance = normalDistance(b, intersectionSimple);
		/* does collapse so we create an event
		 * and add it to the queue
		 **/
		LOG(INFO) << "YES ofr " << aIdx << "," << bIdx << "," << cIdx << " at " << intersectionSimple << " with dist: " << distance;
		return Event(distance,intersectionSimple,aIdx,bIdx,cIdx,it);
	} else if( intersectionSimple != INFPOINT) {
		LOG(INFO) << "/// some special case! " << aIdx << ", " << bIdx << ", " << cIdx;
	}

	return Event(MAX,INFPOINT,aIdx,bIdx,cIdx,it);
}


void Wavefront::ChainDecomposition() {
	/* assuming CCW orientation of polygon */
	auto edgeIt = data.findEdgeWithVertex(data.bbox->monMin);

	lowerChain.emplace_back(edgeIt->id);
	do {
		edgeIt = data.cNext(edgeIt);
		lowerChain.emplace_back(edgeIt->id);
	} while(!edgeIt->has(data.bbox->monMax.id));

	do {
		edgeIt = data.cNext(edgeIt);
		upperChain.emplace_back(edgeIt->id);
	} while(!edgeIt->has(data.bbox->monMin.id));

}

ul Wavefront::addArcRay(const ul& nodeAIdx, const ul& edgeLeft, const ul& edgeRight, const Ray& ray) {
	auto& nodeA = nodes[nodeAIdx];
	auto arcIdx = arcList.size();

	const Segment seg = restrictRay(ray);

	arcList.emplace_back(Arc(
			ArcType::RAY,
			nodeAIdx,
			MAX,
			edgeLeft,
			edgeRight,
			arcList.size(),
			seg
	));

	nodeA.arcs.emplace_back(arcIdx);
	LOG(INFO) << "+/ adding ray: " << arcIdx << " -- " << arcList.back();
	return arcIdx;
}

ul Wavefront::addArc(const ul& nodeAIdx, const ul& nodeBIdx, const ul& edgeLeft, const ul& edgeRight) {
	auto& nodeA = nodes[nodeAIdx];
	auto& nodeB = nodes[nodeBIdx];
	auto arcIdx = arcList.size();
	arcList.emplace_back(Arc(
			ArcType::NORMAL,
			nodeAIdx,
			nodeBIdx,
			edgeLeft,
			edgeRight,
			arcList.size(),
			Segment(nodeA.point, nodeB.point)
	));
	nodeA.arcs.emplace_back(arcIdx);
	nodeB.arcs.emplace_back(arcIdx);
	LOG(INFO) << "++ adding arc: " << arcIdx;
	return arcIdx;
}

Arc* Wavefront::getRightmostArcEndingAtNode(const Node& node, Arc *currentArc) {
	Arc *ret = currentArc;
	NT x = currentArc->point(0).x();
	for(auto arcIdx : node.arcs) {
		if(arcIdx == currentArc->id) {continue;}
		Arc* arc = getArc(arcIdx);
		if(arc->secondNodeIdx == node.id) {
			if(x < arc->point(0).x()) {
				x = arc->point(0).x();
				ret = arc;
			}
		}
	}
	return ret;
}

ul Wavefront::getOutgoingArc(const Node& node) {
	for(ul arcIdx : node.arcs) {
		Arc* arc = getArc(arcIdx);
		if(arc->firstNodeIdx == node.id) {
			return arcIdx;
		}
	}
	return MAX;
}

//void Wavefront::removeOutgointArcsOnNode(Node& node) {
//	std::vector<ul> toremove;
//	for(auto arcIdx : node.arcs) {
//		Arc& arc = arcList[arcIdx];
//		if(arc.firstNodeIdx == node.id) {
//			arc.disable();
//			toremove.push_back(arcIdx);
//		}
//	}
//	for(auto idx : toremove) {
//		node.removeArc(idx);
//	}
//}

Segment Wavefront::restrictRay(const Ray& ray) {
	Point Pa = ray.source();
	Point Pb;

	if(state == STATE::UPPER) {
		if(data.bbox->yMin.p.y() > Pa.y()) {
			Pb = Pa;
		} else {
			NT Pb_x = ray.supporting_line().x_at_y(data.bbox->yMin.p.y());
			Pb = Point(Pb_x,data.bbox->yMin.p.y());
		}
	} else {
		if(data.bbox->yMax.p.y() < Pa.y()) {
			Pb = Pa;
		} else {
			NT Pb_x = ray.supporting_line().x_at_y(data.bbox->yMax.p.y());
			Pb = Point(Pb_x,data.bbox->yMax.p.y());
		}
	}

	return Segment(ray.source(),Pb);
}

void Wavefront::addNewNodefromEvent(const Event& event) {
	ul nodeIdx  = nodes.size();
	auto& paths = pathFinder[event.mainEdge];

	Point& Pa = getNode(paths.a)->point; Point& Pb = getNode(paths.b)->point;

	/* if this is already done, i.e., left and/or right path ends at a node of the event */
	if( Pa != event.eventPoint && Pb != event.eventPoint ) {
		/* a classical event to be handled */
		LOG(INFO) << "event point before adding node " << event.eventPoint;
		nodeIdx = addNode(event.eventPoint,event.eventTime);

		addArc(paths.a,nodeIdx,event.leftEdge,event.mainEdge);
		addArc(paths.b,nodeIdx,event.mainEdge,event.rightEdge);

	} else {
		/* at least one point is equal */

		bool aEqual = (Pa == event.eventPoint);
		bool bEqual = (Pb == event.eventPoint);

		if(aEqual && bEqual) {
			nodeIdx = paths.a;
		} else if(aEqual) {
			/* so we use the left referenced node and only create a new arc for the right side */
			nodeIdx = paths.a;
			addArc(paths.b,nodeIdx,event.mainEdge,event.rightEdge);
		} else if(bEqual) {
			/* so we use the left referenced node and only create a new arc for the left side */
			nodeIdx = paths.b;
			addArc(paths.a,nodeIdx,event.leftEdge,event.mainEdge);
		}
	}

	/* update path finder for left and right edge */
	pathFinder[event.leftEdge].b  = nodeIdx;
	pathFinder[event.rightEdge].a = nodeIdx;
}

ul Wavefront::getNextArcIdx(const ul& path, bool forward, ul edgeIdx) {
	if(path >= arcList.size()) {return MAX;}
	assert(path < arcList.size());
	auto* arc = getArc(path);
	if(forward && arc->isRay()) {return MAX;}
	auto& node = (forward) ? nodes[arc->secondNodeIdx] : nodes[arc->firstNodeIdx];
	for(auto a : node.arcs) {
		if( a != path ) {
			if( forward
					&& arcList[a].firstNodeIdx == arc->secondNodeIdx
					&& !arcList[a].isDisable()
					&& (arcList[a].leftEdgeIdx == edgeIdx || arcList[a].rightEdgeIdx ==edgeIdx)
			) {
				return a;
			} else if( !forward
					&& arcList[a].secondNodeIdx == arc->firstNodeIdx
					&& !arcList[a].isDisable()
					&& (arcList[a].leftEdgeIdx == edgeIdx || arcList[a].rightEdgeIdx ==edgeIdx)
			) {
				return a;
			}
		}
	}
	return MAX;
}


void Wavefront::printChain(const Chain& chain) const {
	std::stringstream ss;
	ss <<  "chain links: ";
	for(auto l : chain) {
		ss << l << " ";
	}
	LOG(INFO) << ss.str();
}

//void Wavefront::printEvents() const {
//	std::stringstream ss;
//	ss << "eventlist: " << std::endl;
//	for(auto t : eventTimes) {
//		auto e = events[t.edgeIdx];
//		ss << e << std::endl;
//	}
//	LOG(INFO) << ss.str();
//}

