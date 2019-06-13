#include "Wavefront.h"

std::ostream& operator<< (std::ostream& os, const MonotonePathTraversal& path) {
	os << "path-edge (" << path.edgeIdx << ") - current:" << path.currentArcIdx << " opposite: " << path.oppositeArcIdx;
	return os;
}


void Wavefront::InitializeEventsAndPathsPerEdge() {
	/* set up empty events for every edge;
	* set up initial target node for pathfinder
	**/
	for(auto e : data.getPolygon()) {
		events.push_back(Event());
		pathFinder.push_back(e);
	}
}

void Wavefront::InitializeNodes() {
	/* create all terminal nodes of skeleton (vertices of input) */
	for(auto v : data.getVertices()) {
		Node n(NodeType::TERMINAL, v, 0);
		nodes.push_back(n);
	}
}

bool Wavefront::InitSkeletonQueue(Chain& chain, PartialSkeleton& skeleton) {
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {return true;}

	auto chainIterator = chain.begin();
	/* first edge defines an unbounded face in the skeleton induced graph */
	uint aEdgeIdx = *chainIterator;
	++chainIterator;
	uint bEdgeIdx = *chainIterator;
	ChainRef it = ChainRef(chainIterator);
	++chainIterator;
	uint cEdgeIdx = *chainIterator;

	/************************************/
	/* 	filling the priority queue 		*/
	/************************************/
	do {
		cEdgeIdx = *chainIterator;
		/* create Event and add it to the queue */
		auto event = getEdgeEvent(aEdgeIdx,bEdgeIdx,cEdgeIdx,it);
		if(event.isEvent()) {
			events[event.mainEdge()] = event;
			auto te = TimeEdge(event.eventTime,event.mainEdge());
			eventTimes.insert(te);
			std::cout << event << std::endl;
		}

		/* iterate over the chainIterator */
		it = chainIterator;
		++chainIterator;
		aEdgeIdx = bEdgeIdx;
		bEdgeIdx = cEdgeIdx;
	} while(chainIterator != chain.end());


	currentTime = 0;

	return true;
}

bool Wavefront::ComputeSkeleton(bool lower) {
	auto& chain     = (lower) ? getLowerChain()	: getUpperChain();
	auto& skeleton  = (lower) ? lowerSkeleton 	: upperSkeleton;
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {return true;}

	/************************************/
	/* 	filling the priority queue 		*/
	/************************************/
	if(!InitSkeletonQueue(chain,skeleton)) {return false;}

	/*********************************************/
	/* compute skeleton by working through queue */
	/*********************************************/
	currentTime = 0;
	while(!eventTimes.empty()) {
		SingleDequeue(chain,skeleton);
	}

	/***********************************************************************/
	/* construct rays from remaining edges in chain, i.e,. unbounded faces */
	/***********************************************************************/
	if(!FinishSkeleton(chain,skeleton)) {return false;}

	return true;
}


/**
 * computes a single skeleton event from the queue,
 * returns false if queue is empty
 * */
bool Wavefront::ComputeSingleSkeletonEvent(bool lower) {
	auto& chain     = (lower) ? getLowerChain()	: getUpperChain();
	auto& skeleton  = (lower) ? lowerSkeleton 	: upperSkeleton;

	while(!eventTimes.empty() && !SingleDequeue(chain,skeleton));

	return !eventTimes.empty();
}

bool Wavefront::SingleDequeue(Chain& chain, PartialSkeleton& skeleton) {
	std::cout << "SingleDequeue :: ";
	auto etIt  = eventTimes.begin();
	auto event = &events[etIt->edgeIdx];
	auto eventTime = etIt->time;
	eventTimes.erase(etIt);

	if(currentTime <= eventTime) {

		currentTime = eventTime;

		std::vector<Event*> multiEventStack;
		multiEventStack.push_back(event);
		while(!eventTimes.empty() && eventTime == eventTimes.begin()->time) {
			/* check for multi-events */
			auto etCheck = eventTimes.begin();
			event = &events[etCheck->edgeIdx];
			multiEventStack.push_back(event);
			eventTimes.erase(etCheck);
		}

		if(multiEventStack.size() > 1) {
			/********************************************************************************/
			/* ---------------------------- MULTI EVENTS HERE ------------------------------*/
			/********************************************************************************/

			LOG(INFO) << "HANDLE MULTIPLE EVENTS (equal TIME)!";
			std::map<Point,uint> pointToIndex;
			/* we store a list of events per point (projected on the monotonicity line)  */
			std::vector<std::vector<Event*>> eventsPerPoint;
			for(auto e : multiEventStack) {
				Point p = data.pointOnMonotonicityLine(e->eventPoint);

				/* build map point -> list of events */
				auto it = pointToIndex.find(p);
				if(it != pointToIndex.end()) {
					eventsPerPoint[it->second].push_back(e);
				} else {
					pointToIndex.insert(std::pair<Point,uint>(p,eventsPerPoint.size()));
					std::vector<Event*> list = {e};
					eventsPerPoint.push_back( list );
				}
			}

			for(auto eventList : eventsPerPoint) {
				LOG(INFO) << "ME " << *event;
				HandleMultiEvent(chain,skeleton,eventList);
			}

		} else {
			/********************************************************************************/
			/* --------------------------- SINGLE EDGE EVENTS ------------------------------*/
			/********************************************************************************/
			LOG(INFO) << "SE " << *event;
			HandleSingleEdgeEvent(chain,skeleton,event);
		}

		return true;
	}
	return false;
}



/* due to the monotonicity we should only have two scenarios:
 * (i) this amounts to exactly one point with multiple collapsing edges:
 * --> then we continue with the bisector between the first and last involved
 *     edge.
 * (ii)  otherwise a vertical bisector line is involved, then we should have
 *       exactly two points:
 * --> after adding both points (connected with a vertical segment) we apply
 *     (i) for the 'upper' (or higher) point. */
void Wavefront::HandleMultiEvent(Chain& chain, PartialSkeleton& skeleton,std::vector<Event*> eventList) {
	if(eventList.size() == 1) {
		LOG(INFO) << "size 1 ";
		HandleSingleEdgeEvent(chain,skeleton,eventList[0]);
	} else {
		std::set<Point> points;
		std::set<int,std::less<int> > eventEdges;
		LOG(INFO) << "events:";
		for(auto e : eventList) {
			LOG(INFO) << *e;
			points.insert(e->eventPoint);
			eventEdges.insert(e->edges[0]);
			eventEdges.insert(e->edges[1]);
			eventEdges.insert(e->edges[2]);
		}

		if(points.size() > 1) {
			/* scenario (ii) : ONLY TWO POINTS SHOULD BE POSSIBLE!
			 * this scenario implies a vertical segment, i.e., perpendicular to the
			 * monotonicity line. This means only two event moints can occur on this
			 * line, otherwise the polygon can not be monotone */
			assert(points.size() < 3);

			/* ensure A is the lower point in resp. to monot. line */
			auto it = points.begin();
			Point A = *it; ++it;
			Point B = *it;

			std::cout << std::boolalpha << data.isAbove(A,B);
			std::cout << std::boolalpha << isLowerChain(chain);

			if( ( data.isAbove(A,B) &&  isLowerChain(chain)) ||
				(!data.isAbove(A,B) && !isLowerChain(chain))) {
				std::swap(A,B);
			}

			std::vector<Event*> partEventListA, partEventListB;
			for(auto e : eventList) {
				if(e->eventPoint == B) {
					partEventListB.push_back(e);
				} else {
					partEventListA.push_back(e);
				}
			}

			assert(partEventListA.size() == 1);

			/* handling the 'lower' eventpoint changes the other event, thus
			 * we only handle this lower event */
			HandleSingleEdgeEvent(chain,skeleton,partEventListA[0]);
		} else {
			/* scenario (i) */
			LOG(INFO) << "scenario (i)";
			HandleMultiEdgeEvent(chain,skeleton,eventList);
		}
	}
}

void Wavefront::HandleMultiEdgeEvent(Chain& chain, PartialSkeleton& skeleton, std::vector<Event*> eventList) {
	LOG(INFO) << "HandleMultiEdgeEvent! (TO-BE-TESTED!)";

	auto nodeIdx = nodes.size();
	auto anEvent = eventList[0];

	/* add the single node, all arcs connect to this node */
	auto node = Node(NodeType::NORMAL,anEvent->eventPoint,anEvent->eventTime);
	nodes.push_back(node);
	skeleton.push_back(nodeIdx);

	bool singleLeft = true;
	for(auto event : eventList) {
		auto idx = event->mainEdge();
		auto paths = pathFinder[idx];
		if(singleLeft) {
			addArc(paths[0],nodeIdx,event->leftEdge(),event->mainEdge());
			singleLeft = false;
		}
		addArc(paths[1],nodeIdx,event->mainEdge(),event->rightEdge());

		/* update path finder for left and right edge */
		pathFinder[idx][1] = nodeIdx;
		pathFinder[idx][0] = nodeIdx;
	}

//	updateNeighborEdgeEvents(*eventList[0],chain);
//	updateNeighborEdgeEvents(*lastEvent,chain);
	auto chainIt = chain.begin();
	for(auto event : eventList) {
		/* remove this edges from the chain (wavefront) */
		chainIt = chain.erase(event->chainEdge);
		disableEdge(event->mainEdge());
	}
	--chainIt;

	auto idxA = *(--chainIt);
	auto idxB = *(++chainIt);
	auto idxC = *(++chainIt);
	auto idxD = *(++chainIt);
	--chainIt;
	--chainIt;
	ChainRef it1 = ChainRef(chainIt);
	ChainRef it2 = ChainRef(++chainIt);
	auto e1 = getEdgeEvent(idxA,idxB,idxC,it1);
	auto e2 = getEdgeEvent(idxB,idxC,idxD,it2);

	LOG(INFO) << idxA << " " << idxB << " " << idxC << " " << idxD;
	LOG(INFO) << e1;
	LOG(INFO) << e2;

	pathFinder[idxB][1] = nodeIdx;
	pathFinder[idxC][0] = nodeIdx;

	updateInsertEvent(e1);
	updateInsertEvent(e2);
}


void Wavefront::HandleSingleEdgeEvent(Chain& chain, PartialSkeleton& skeleton, Event* event) {
	/* build skeleton from event */
	addNewNodefromEvent(*event,skeleton);

	/* check neighbours for new events, and back into the queue */
	/* edges B,C are the two edges left, right of the event edge,
	 * A,D their respective neighbours */
	updateNeighborEdgeEvents(*event,chain);

	/* remove this edge from the chain (wavefront) */
	chain.erase(event->chainEdge);
	disableEdge(event->mainEdge());
}

bool Wavefront::FinishSkeleton(Chain& chain, PartialSkeleton& skeleton) {
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {return true;}

	while(!eventTimes.empty()) {
		SingleDequeue(chain,skeleton);
	}

	uint aEdgeIdx, bEdgeIdx;
	auto chainIterator = chain.begin();
	/***********************************************************************/
	/* construct rays from remaining edges in chain, i.e,. unbounded faces */
	/***********************************************************************/
	if(chain.size() > 1) {
		chainIterator = chain.begin();
		aEdgeIdx = *chainIterator; ++chainIterator;
		do {
			bEdgeIdx = *chainIterator;
			/* last node on path of both edges must be the same, get that node */
			auto endNodeIdx = pathFinder[aEdgeIdx][1];
			auto node = &nodes[endNodeIdx];
			Ray bis;

			/* compute bisector between the two edges */
			auto bisRet = constructBisector(aEdgeIdx,bEdgeIdx);
			if(bisRet.type == BisType::RAY) {
				bis = Ray(node->point,bisRet.ray.direction());
			} else {
				Line l(bisRet.line);
				Point pRef(data.monotonicityLine.point()+l.to_vector());
				if( isLowerChain(chain) && data.monotonicityLine.has_on_negative_side(pRef)) {
					/* vector points below monotonicity line */
					l = l.opposite();
				}
				if(!isLowerChain(chain) && data.monotonicityLine.has_on_positive_side(pRef)) {
					/* vector points above monotonicity line */
					l = l.opposite();
				}

				bis = Ray(node->point,bisRet.line.direction());
			}

			/* need to know if upper or lower chain! To distinguish bisector rays that lead up or down! */
			auto check = normalDistance(data.getEdge(aEdgeIdx).supporting_line(),node->point + bis.to_vector());
			if(check < node->time) {
				bis = bis.opposite();
			}

			addArcRay(endNodeIdx,aEdgeIdx,bEdgeIdx,bis);

			/* iterate over remaining chain */
			aEdgeIdx = bEdgeIdx;
		} while(++chainIterator != chain.end());
	}

	return true;
}

void Wavefront::updateNeighborEdgeEvents(const Event& event, const Chain& chain) {
	uint edgeA, edgeB, edgeC, edgeD;
	edgeB = event.leftEdge();
	edgeC = event.rightEdge();
	std::cout << event;
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

void Wavefront::updateInsertEvent(const Event& event) {
	/* check if edge has already an event in the queue */
	auto currentEvent =  events[event.mainEdge()];

	if(currentEvent.eventTime > 0) {
		/* find remove timeslot from event times */
		auto teOld = TimeEdge(currentEvent.eventTime,currentEvent.mainEdge());
		auto item  = eventTimes.lower_bound(teOld);
		if(item != eventTimes.end() && item->edgeIdx == event.mainEdge()) {
			eventTimes.erase(item);
		}
	}

	assert(event.mainEdge() == *event.chainEdge);

	events[event.mainEdge()] = Event(event);
	if(event.eventPoint != INFPOINT) {
		auto te = TimeEdge(event.eventTime,event.mainEdge());
		eventTimes.insert(te);
	}
}

bool Wavefront::hasParallelBisector(const Event& event) const {
	auto lA = data.getEdge(event.leftEdge()).supporting_line();
	auto lB = data.getEdge(event.mainEdge()).supporting_line();
	auto lC = data.getEdge(event.rightEdge()).supporting_line();
	return CGAL::parallel(lA,lB) || CGAL::parallel(lB,lC) || CGAL::parallel(lA,lC);
}

Event Wavefront::getEdgeEvent(const uint& aIdx, const uint& bIdx, const uint& cIdx, const ChainRef& it) const {
	Event e;
//	Ray abRay, bcRay;

	/* compute bisector from edges */
	auto abBis = constructBisector(aIdx, bIdx);
	auto bcBis = constructBisector(bIdx, cIdx);

//	/* in case of rays all is good,
//	 * if we get lines as bisectors we construct rays by using 'bIdx' as the 'edge in the middle'
//	 * */
//	auto pAB = nodes[pathFinder[aIdx][1]].point;
//	if(abBis.type == BisType::RAY) {
//		abRay = Ray(pAB,abBis.ray.direction());
//	} else {
//		Line l(bcBis.line);
//		Line refLine(data.getEdge(bIdx).supporting_line());
//		if(normalDistance(refLine,pAB) > normalDistance(refLine,pAB+l.to_vector())) {
//			l = l.opposite();
//			bcRay = Ray(pAB,l.direction());
//		} {
//			bcRay = Ray(pAB,l.direction());
//		}
//	}
//
//	auto pBC = nodes[pathFinder[bIdx][1]].point;
//	if(bcBis.type == BisType::RAY) {
//		bcRay = Ray(pBC,bcBis.ray.direction());
//	} else {
//		Line l(bcBis.line);
//		Line refLine(data.getEdge(bIdx).supporting_line());
//		if(normalDistance(refLine,pBC) > normalDistance(refLine,pBC+l.to_vector())) {
//			l = l.opposite();
//			bcRay = Ray(pBC,l.direction());
//		} {
//			bcRay = Ray(pBC,l.direction());
//		}
//	}


	/* compute bisector intersection, this is the collapse
	 * time of the middle edge (b) 'edge-event' for b  */
	std::cout << "bi " << std::endl; fflush(stdout);
	auto intersection = intersectElements(abBis.supporting_line(), bcBis.supporting_line());
	std::cout << "ai " << std::endl; fflush(stdout);

	Line b(data.getEdge(bIdx).supporting_line());
	if(intersection != INFPOINT && b.has_on_positive_side(intersection)) {
		auto distance = normalDistance(b, intersection);
		assert(distance > 0);
		/* does collapse so we create an event
		 * and add it to the queue
		 **/
		e = Event(distance,intersection,aIdx,bIdx,cIdx,it);
	} else {
		if(CGAL::collinear(abBis.point(0),abBis.point(1),bcBis.point(1))) {
			LOG(INFO) << "bisectors are collinear!";
			e = Event(0,INFPOINT,aIdx,bIdx,cIdx,it);
		} else {
			e = Event(0,INFPOINT,aIdx,bIdx,cIdx,it);
		}
	}
	std::cout << e << " ret event. "; fflush(stdout);
	return e;
}

Bisector Wavefront::constructBisector(const uint& aIdx, const uint& bIdx) const {
	Line a(data.getEdge(aIdx).supporting_line());
	Line b(data.getEdge(bIdx).supporting_line());

	Point intersectionA = intersectElements(a, b);

	/* classical bisector (unweighted) */
	if( data.w(aIdx) == 1 && data.w(bIdx) == 1) {
		if(intersectionA != INFPOINT) {
			Point aP, bP, cP;
			aP = data.eA(aIdx);
			bP = intersectionA;
			cP = data.eB(bIdx);

			Line bisLine = CGAL::bisector(a,b.opposite());
			Point pBis = bisLine.point();
			Ray bis(intersectionA,pBis);

			if( !a.has_on_positive_side(pBis) || !b.has_on_positive_side(pBis) ) {
				bis = bis.opposite();
			}
			return Bisector(bis);
		} else {
			if(CGAL::collinear(a.point(0),a.point(1),b.point(0))) {
				return Bisector(Ray(a.point(0),a.perpendicular(a.point(0)).to_vector()));
			} else {
				Line bisLine = CGAL::bisector(a,b.opposite());
				auto bis = Bisector(bisLine);
				bis.setPerpendicular(true);
				std::cout << bisLine; fflush(stdout);
				return bis;
			}
		}
	} else {
		/* weighted bisector */
		LOG(INFO) << "weighted bisector!";
		if(intersectionA != INFPOINT) {
			Point bP = intersectionA;
			Vector aN = a.perpendicular(bP).to_vector();
			Vector bN = b.perpendicular(bP).to_vector();

			aN /= CGAL::sqrt(aN.squared_length());
			bN /= CGAL::sqrt(bN.squared_length());

			aN *= data.w(aIdx);
			bN *= data.w(bIdx);

			Line aOffsetLine    = Line( bP + aN , a.direction() );
			Line bOffsetLine    = Line( bP + bN , b.direction() );
			Point intersectionB = intersectElements(aOffsetLine, bOffsetLine);

			return Bisector(Ray(intersectionA,intersectionB));

		} else {
			LOG(INFO) << "parallel weighted bisector (TODO)!";

			if(!CGAL::collinear(a.point(0),a.point(1),b.point(0))) {
				Point Pa = a.point(0);
				Line l = a.perpendicular(Pa);
				Point Pb = intersectElements(l,b);
				Vector aN = l.to_vector();
				aN /= CGAL::sqrt(aN.squared_length());
				Vector bN = -aN;

				auto aDir = aN.perpendicular(CGAL::LEFT_TURN);

				Point Pa_off = Pa + aDir;
				Point Pb_off = Pb + aDir;

				aN *= data.w(aIdx);
				bN *= data.w(bIdx);

				Point Pa2 = Pa_off + aN;
				Point Pb2 = Pb_off + bN;

				Ray R1 = Ray(Pa,Pa2);
				Ray R2 = Ray(Pa,Pa2);

				Point wMidPoint = intersectElements(R1,R2);

				Ray bis(wMidPoint,-a.direction());
				Edge e = data.confineRayToBBox(bis);

				bis = Ray(e.target(),wMidPoint);
				return Bisector(bis);
			} else {
				LOG(ERROR) << "parallel edges -> (TODO)!";
				assert(false);
				return Bisector(Ray());
			}
		}
	}
}
Bisector Wavefront::getBisectorWRTMonotonicityLine(const Bisector& bisector) const {
	Bisector bis(bisector);

	Line lp(ORIGIN,data.perpMonotonDir);
	Point p(ORIGIN+bis.to_vector());

	if(lp.has_on_positive_side(p)) {
		bis.changeDirection();
	} else if(lp.has_on_boundary(p)) {
		bis.setPerpendicular(true);
		LOG(WARNING) << "WARNUNG: ... bisector is perpendicular to monotonicity line.";
	}

	return bis;
}

void Wavefront::ChainDecomposition() {
	Chain lc, uc;

	uint minVertex = data.bbox.monotoneMinIdx;
	uint maxVertex = data.bbox.monotoneMaxIdx;

	uint startLEI, endLEI;
	uint startUEI, endUEI;

	uint polygonSize = data.getPolygon().size();

	/** find chain index in polygon (assuming CCW orientation) */
	for(uint i = 0; i < polygonSize; ++i) {
		if(data.e(i)[0] == minVertex) {
			startLEI = i;
		}
		if(data.e(i)[1] == maxVertex) {
			endLEI = i;
		}

		if(data.e(i)[0] == maxVertex) {
			startUEI = i;
		}
		if(data.e(i)[1] == minVertex) {
			endUEI = i;
		}
	}

	if(startLEI == endLEI) {
		lc.push_back(startLEI);
	} else {
		for(auto i = startLEI;
				(startLEI < endLEI   && i <= endLEI) ||
				(endLEI   < startLEI && i <= polygonSize+endLEI);
				++i) {
			lc.push_back(i%polygonSize);
		}
	}

	if(startUEI == endUEI) {
		uc.push_back(startUEI);
	} else {
		for(auto i = startUEI;
				(startUEI < endUEI   && i <= endUEI) ||
				(endUEI   < startUEI && i <= polygonSize+endUEI);
				++i) {
			uc.push_back(i%polygonSize);
		}
	}

	/* store the indices for later use as we use the chains as wavefront */
	startLowerEdgeIdx = startLEI;
	endLowerEdgeIdx   = endLEI;
	startUpperEdgeIdx = startUEI;
	endUpperEdgeIdx   = endUEI;

	lowerChain = lc;
	upperChain = uc;


	/*  some DEBUG output  */
	std::cout << "upper chain: ";
	for(auto e : upperChain) {
		std::cout << e << " ";
	}
	std::cout << std::endl << "lower chain: ";
	for(auto e : lowerChain) {
		std::cout << e << " ";
	}
	std::cout << std::endl;
}

uint Wavefront::addArcRay(const uint& nodeAIdx, const uint& edgeLeft, const uint& edgeRight, const Ray& ray) {
	auto nodeA = &nodes[nodeAIdx];
	Arc arc(ArcType::RAY, nodeAIdx, edgeLeft, edgeRight, ray);
	auto arcIdx = arcList.size();
	arcList.push_back(arc);
	nodeA->arcs.push_back(arcIdx);

	return arcIdx;
}

uint Wavefront::addArc(const uint& nodeAIdx, const uint& nodeBIdx, const uint& edgeLeft, const uint& edgeRight) {
	auto nodeA = &nodes[nodeAIdx];
	auto nodeB = &nodes[nodeBIdx];
	Arc arc(ArcType::NORMAL, nodeAIdx, nodeBIdx, edgeLeft, edgeRight, Edge(nodeA->point, nodeB->point));
	auto arcIdx = arcList.size();
	arcList.push_back(arc);
	nodeA->arcs.push_back(arcIdx);
	nodeB->arcs.push_back(arcIdx);

	return arcIdx;
}

void Wavefront::addNewNodefromEvent(const Event& event, PartialSkeleton& skeleton) {
	uint nodeIdx = nodes.size();
	auto paths 	 = pathFinder[event.mainEdge()];

	/* if this is already done, i.e., left and/or right path ends at a node of the event */
	if(nodes[paths[0]].point == event.eventPoint && nodes[paths[1]].point == event.eventPoint) {
		nodeIdx = paths[0];
	} else if(nodes[paths[0]].point == event.eventPoint) {
		/* so we use the left referenced node and only create a new arc for the right side */
		nodeIdx = paths[0];
		addArc(paths[1],nodeIdx,event.mainEdge(),event.rightEdge());
	} else if(nodes[paths[1]].point == event.eventPoint) {
		/* so we use the left referenced node and only create a new arc for the left side */
		nodeIdx = paths[1];
		addArc(paths[0],nodeIdx,event.leftEdge(),event.mainEdge());
	} else {
		/* a classical event to be handled */
		auto node = Node(NodeType::NORMAL,event.eventPoint,event.eventTime);
		nodes.push_back(node);
		skeleton.push_back(nodeIdx);

		addArc(paths[0],nodeIdx,event.leftEdge(),event.mainEdge());
		addArc(paths[1],nodeIdx,event.mainEdge(),event.rightEdge());
	}

	/* update path finder for left and right edge */
	pathFinder[event.leftEdge()][1]  = nodeIdx;
	pathFinder[event.rightEdge()][0] = nodeIdx;
}

bool Wavefront::nextMonotoneArcOfPath(MonotonePathTraversal& path) {
	LOG(INFO) << "nextMonotoneArcOfPath";
	std::cout << path << std::endl;

	if(path.done()) {return false;}

	auto currentArc  = &arcList[path.currentArcIdx];
	auto oppositeArc = &arcList[path.oppositeArcIdx];

	/* check if rightmost end-point of both arcs is the same */
	if(currentArc->adjacent(*oppositeArc)) { // && getRightmostNodeIdxOfArc(currentArc) == getRightmostNodeIdxOfArc(oppositeArc))  {
		path.currentArcIdx = path.oppositeArcIdx;
		LOG(INFO) << "current and opposite are adjacent";
		return true;
	//} else if(currentArc.adjacent(oppositeArc)) {

	} else if(isArcLeftOfArc(*oppositeArc,*currentArc)) { // && getLeftmostNodeIdxOfArc(currentArc) != getLeftmostNodeIdxOfArc(oppositeArc) ) {
		/* opposite arcs left endpoint is to the left of the current arc ones */
		std::cout << "swap " << path << " --> ";
		path.swap();
		std::cout << path << std::endl;
		return true;
	} else {
		/* step to the next arc to the right of current arc */
		uint nextArcIdx = getNextArcIdx(path,*currentArc);

		if(nextArcIdx != MAX) {
			LOG(INFO) << "next arc " << nextArcIdx << " found";
			path.currentArcIdx = nextArcIdx;
			currentArc  = &arcList[path.currentArcIdx];
			if(isArcLeftOfArc(*oppositeArc,*currentArc)) {
				path.swap();
			}
			LOG(INFO) << "next arc " << path.currentArcIdx << " found";
			return true;
		} else {
			LOG(ERROR) << "No next arc found!";
			return false;
		}
	}
}

uint Wavefront::getNextArcIdx(const MonotonePathTraversal& path, const Arc& arc) const {
	uint rightNodeIdx = getRightmostNodeIdxOfArc(arc);
	auto rightNode    = nodes[rightNodeIdx];
	uint repeat       = 1;

	/* run twice */
	do{
		for(auto a : rightNode.arcs) {
			if( !path.isAnIndex(a) ) {
				auto arcIt = arcList[a];
				if( liesOnFace(arcIt,path.edgeIdx) ) {
					return a;
				}
			}
		}
		rightNode = nodes[arc.getSecondNodeIdx(rightNodeIdx)];
	} while(repeat-- > 0);

	return MAX;
}

bool Wavefront::isArcLeftOfPoint(const Arc& arc, const Point& point) const {
	auto Idx = getRightmostNodeIdxOfArc(arc);
	auto Na = &nodes[Idx];
	if(arc.isEdge()) {
		return data.monotoneSmaller(Na->point,point);
	} else {
		assert(arc.isRay());
		return data.monotoneSmaller(Na->point,point) && !data.rayPointsLeft(arc.ray);
	}
}

bool Wavefront::isArcLeftOfArc(const Arc& arcA, const Arc& arcB) const {
	return isArcLeftOfArc(data.monotonicityLine,arcA,arcB);
}

bool Wavefront::isArcLeftOfArc(const Line& line, const Arc& arcA, const Arc& arcB) const {
	auto NaIdx = getLeftmostNodeIdxOfArc(arcA);
	auto NbIdx = getLeftmostNodeIdxOfArc(arcB);
	auto Na = &nodes[NaIdx];
	auto Nb = &nodes[NbIdx];

	/* left endpoints are adjacent, check right endpoints */
	if(NaIdx == NbIdx) {
		NaIdx = getRightmostNodeIdxOfArc(arcA);
		NbIdx = getRightmostNodeIdxOfArc(arcB);
		Na = &nodes[NaIdx];
		Nb = &nodes[NbIdx];
	}

	bool pointAmonotoneSmaller = data.monotoneSmaller(line,Na->point,Nb->point);

	if(arcA.isEdge() && arcB.isEdge()) {
		/* 1st: both arcs are edges */
		return pointAmonotoneSmaller;
	} else if( (arcA.isEdge() && arcB.isRay()) || (arcA.isRay() && arcB.isEdge()) ) {
		/* 2nd: one edge one ray */
		auto& ray  = (arcA.isRay())  ? arcA : arcB;
		bool rayPointsLeft = data.rayPointsLeft(ray.ray);

		if(    ( rayPointsLeft && arcA.isRay())
			|| (!rayPointsLeft && pointAmonotoneSmaller)
		) {
			return true;
		} else {
			return false;
		}
	} else {
		/* 3rd: both arcs are rays */
		assert(arcA.isRay());
		assert(arcB.isRay());

		bool rayAPointsLeft = data.rayPointsLeft(arcA.ray);
		bool rayBPointsLeft = data.rayPointsLeft(arcB.ray);

		if( (rayAPointsLeft && rayBPointsLeft) || (!rayAPointsLeft && !rayBPointsLeft) ) {
			return pointAmonotoneSmaller;
		} else if(rayAPointsLeft) {
			return true;
		} else if(rayBPointsLeft) {
			return false;
		}
	}

	assert(false);
	return false;
}

uint Wavefront::getRightmostNodeIdxOfArc(const Arc& arc) const {
	const auto& Na = nodes[arc.firstNodeIdx];
	if(arc.isEdge()) {
		const auto& Nb = nodes[arc.secondNodeIdx];
		return (data.monotoneSmaller(Na.point,Nb.point)) ? arc.secondNodeIdx : arc.firstNodeIdx;
	} else if (arc.isRay()) {
		return arc.firstNodeIdx;
	} else {
		LOG(ERROR) << "Traversing a disabled arc/ray!";
		return arc.firstNodeIdx;
	}
}

uint Wavefront::getLeftmostNodeIdxOfArc(const Arc& arc) const {
	const auto& Na = nodes[arc.firstNodeIdx];
	if(arc.isEdge()) {
		const auto& Nb = nodes[arc.secondNodeIdx];
		return (data.monotoneSmaller(Na.point,Nb.point)) ? arc.firstNodeIdx : arc.secondNodeIdx;
	} else if (arc.isRay()) {
		return arc.firstNodeIdx;
	} else {
		LOG(ERROR) << "Traversing a disabled arc/ray!";
		return arc.firstNodeIdx;
	}
}

void Wavefront::initPathForEdge(const bool upper, const uint edgeIdx) {
	/* set the upperPath/lowerPath in 'wf' */
	LOG(INFO) << "initPathForEdge " << edgeIdx; fflush(stdout);
	Node& terminalNode   = (upper) ? getTerminalNodeForVertex(data.e(edgeIdx)[0]) : getTerminalNodeForVertex(data.e(edgeIdx)[1]);

	MonotonePathTraversal path;

	if(!terminalNode.arcs.empty()) {
		uint  initialArcIdx  = terminalNode.arcs.front();
		Arc&  initialArc     = arcList[initialArcIdx];

		auto ie = pathFinder[edgeIdx];
		Node& distantNode   = (upper) ? nodes[ie[0]] : nodes[ie[1]];
		uint  distantArcIdx = getPossibleRayIdx(distantNode,edgeIdx);
		std::cout << "distantArcIdx arc idx " << distantArcIdx << std::endl; fflush(stdout);

		if(distantArcIdx == INFINITY || distantArcIdx == initialArcIdx) {
			path = MonotonePathTraversal(edgeIdx,initialArcIdx,initialArcIdx,upper);
		} else {
			Arc&  distantArc    = arcList[distantArcIdx];
			path = (isArcLeftOfArc(initialArc,distantArc)) ? MonotonePathTraversal(edgeIdx,initialArcIdx,distantArcIdx,upper) : MonotonePathTraversal(edgeIdx,distantArcIdx,initialArcIdx,upper);
		}
	} else {
		path = MonotonePathTraversal(edgeIdx,MAX,MAX,upper);
	}

	if(upper) {
		upperPath = path;
	} else {
		lowerPath = path;
	}

	std::cout << path << std::endl;
}

uint Wavefront::getPossibleRayIdx(const Node& node, uint edgeIdx) const {
	uint arcIdx = INFINITY;
	for(auto a : node.arcs) {
		auto& arc = arcList[a];
		if(arc.type == ArcType::RAY && liesOnFace(arc,edgeIdx)) {
			return a;
		} else if(liesOnFace(arc,edgeIdx)) {
			arcIdx = a;
		}
	}
	return arcIdx;
}


bool Wavefront::isArcPerpendicular(const Arc& arc) const {
	Point Aproj = data.monotonicityLine.projection(arc.point(0));
	Point Bproj = data.monotonicityLine.projection(arc.point(1));
	return Aproj == Bproj;
}

/* used for the output, an arc loses its index in its firstNode when the merge is done */
bool Wavefront::isArcInSkeleton(const uint& arcIdx) const {
	auto arc = &arcList[arcIdx];
	auto node = &nodes[arc->firstNodeIdx];
	uint appearenceCnt = 0;
	for(uint i = 0; i < 2; ++i) {
		for(auto idx : node->arcs) {
			if(idx == arcIdx) {
				++appearenceCnt;
			}
		}
		node = &nodes[arc->secondNodeIdx];
	}
	return appearenceCnt == 2;
}

void Wavefront::SortArcsOnNodes() {
	uint i = 0;
	for(auto n : nodes) {
		n.sort(arcList);

		/*	-- DEBUG ONLY --*/
		std::cout << i << " (" << n.point << ") " << n.arcs.size() << ": ";
		for(auto a : n.arcs) {
			std::cout << a << " [";
			auto arc = &arcList[a];
			std::cout << arc->firstNodeIdx << "-" << arc->secondNodeIdx << "] ";

			auto nodeB = &nodes[ arc->firstNodeIdx ];
			if(arc->type != ArcType::RAY && arc->firstNodeIdx == i) {
				nodeB = &nodes[ arc->secondNodeIdx ];
			}

			if(arc->firstNodeIdx == i) {
				std::cout << arc->secondNodeIdx;
			} else {
				std::cout << arc->firstNodeIdx;
			}
			std::cout <<  " (" << nodeB->arcs.size() << ") ";
		}
		std::cout << std::endl;
		/*	-- DEBUG ONLY --*/
		++i;
	}
}

void Wavefront::printChain(const Chain& chain) const {
	std::cout << "chain links: ";
	for(auto l : chain) {
		std::cout << l << " ";
	}
	std::cout << std::endl;
}

void Wavefront::printEvents() const {
	std::cout << "eventlist: " << std::endl;
	for(auto t : eventTimes) {
		auto e = events[t.edgeIdx];
		std::cout << e << std::endl;
	}
}

void Wavefront::reset() {
	nodes.clear();
	events.clear();
	arcList.clear();
	pathFinder.clear();
	upperSkeleton.clear();
	lowerSkeleton.clear();
	upperChain.clear();
	lowerChain.clear();
	InitializeEventsAndPathsPerEdge();
	InitializeNodes();
}
