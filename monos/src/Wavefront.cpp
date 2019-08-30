#include "Wavefront.h"

std::ostream& operator<< (std::ostream& os, const MonotonePathTraversal& path) {
	os << "path-edge (" << path.edgeIdx << ") - current:" << path.currentArcIdx << " opposite: " << path.oppositeArcIdx;
	if(path.iterateAwayFromEdge) {
		os << " away ";
	} else {
		os << " towards ";
	}
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
			events[event.mainEdge] = event;
			auto te = TimeEdge(event.eventTime,event.mainEdge);
			eventTimes.insert(te);
			LOG(INFO) << event;
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
	LOG(INFO) << "SingleDequeue :: ";
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
			if(e->mainEdge == *(e->chainEdge))  {
				LOG(INFO) << "event is still true!";
			}
			LOG(INFO) << *e;

			points.insert(e->eventPoint);
			eventEdges.insert(e->edges[0]);
			eventEdges.insert(e->edges[1]);
			eventEdges.insert(e->edges[2]);
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
				if( ( data.isAbove(A,B) &&  isLowerChain(chain)) ||
					( data.isAbove(B,A) && !isLowerChain(chain))) {
					std::swap(A,B);
				}
			}
			std::vector<Event*> partEventListA, partEventListB;
			for(auto e : eventList) {
				if(e->eventPoint == A) {
					partEventListA.push_back(e);
				} else {
					partEventListB.push_back(e);
				}
			}

			assert(partEventListA.size() >= 1);

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
	LOG(INFO) << "HandleMultiEdgeEvent! (one point, multiple edge collapses)";

	auto nodeIdx = nodes.size();
	auto anEvent = eventList[0];

	/* add the single node, all arcs connect to this node */
	auto node = Node(NodeType::NORMAL,anEvent->eventPoint,anEvent->eventTime);
	nodes.push_back(node);
	skeleton.push_back(nodeIdx);
	LOG(INFO) << "adding node: " << node;

	bool singleLeft = true;
	for(auto event : eventList) {
		auto idx = event->mainEdge;
		auto paths = pathFinder[idx];
		if(singleLeft) {
			Edge eS(getNode(paths[0])->point,getNode(nodeIdx)->point);
			addArc(paths[0],nodeIdx,event->leftEdge,event->mainEdge,eS.is_vertical(),eS.is_horizontal());
			singleLeft = false;
		}
		Edge e(getNode(paths[1])->point,getNode(nodeIdx)->point);
		addArc(paths[1],nodeIdx,event->mainEdge,event->rightEdge,e.is_vertical(),e.is_horizontal());

		/* update path finder for left and right edge */
		pathFinder[idx][1] = nodeIdx;
		pathFinder[idx][0] = nodeIdx;
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
			pathFinder[idxB][1] = nodeIdx;
			pathFinder[idxC][0] = nodeIdx;

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
			LOG(WARNING) << "EQUAL indices A,B,C,D: " << idxA << " " << idxB << " " << idxC << " " << idxD;
		}
	} else {
		LOG(INFO) << "chain is at beginning with size: " << chain.size();
	}
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
	disableEdge(event->mainEdge);
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

			if(bisRet.isGhost()) {
				auto n = nodes[pathFinder[bisRet.eIdxA][1]];
				bisRet.newSource(n.point);
			}

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

			addArcRay(endNodeIdx,aEdgeIdx,bEdgeIdx,bis,bis.is_vertical(),bis.is_horizontal());

			/* iterate over remaining chain */
			aEdgeIdx = bEdgeIdx;
		} while(++chainIterator != chain.end());
	}

	return true;
}

void Wavefront::updateNeighborEdgeEvents(const Event& event, const Chain& chain) {
	uint edgeA, edgeB, edgeC, edgeD;
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

void Wavefront::updateInsertEvent(const Event& event) {
	/* check if edge has already an event in the queue */
	auto currentEvent =  events[event.mainEdge];

	if(currentEvent.eventTime > 0) {
		/* find remove timeslot from event times */
		auto teOld = TimeEdge(currentEvent.eventTime,currentEvent.mainEdge);
		auto item  = eventTimes.lower_bound(teOld);
		if(item != eventTimes.end() && item->edgeIdx == event.mainEdge) {
			eventTimes.erase(item);
		}
	}

	assert(event.mainEdge == *event.chainEdge);

	events[event.mainEdge] = Event(event);
	if(event.eventPoint != INFPOINT) {
		auto te = TimeEdge(event.eventTime,event.mainEdge);
		eventTimes.insert(te);
	}
}

bool Wavefront::hasParallelBisector(const Event& event) const {
	auto lA = data.getEdge(event.leftEdge).supporting_line();
	auto lB = data.getEdge(event.mainEdge).supporting_line();
	auto lC = data.getEdge(event.rightEdge).supporting_line();
	return CGAL::parallel(lA,lB) || CGAL::parallel(lB,lC) || CGAL::parallel(lA,lC);
}

Event Wavefront::getEdgeEvent(const uint& aIdx, const uint& bIdx, const uint& cIdx, const ChainRef& it) const {
	Event e;

	/* compute bisector from edges */
	auto abBis = constructBisector(aIdx, bIdx);
	auto bcBis = constructBisector(bIdx, cIdx);

	LOG(INFO) << "bisectors before correction " << aIdx << "/" << bIdx;
	LOG(INFO) << abBis;
	LOG(INFO) << "bisectors before correction " << bIdx << "/" << cIdx;
	LOG(INFO) << bcBis;

	/* in case of 'ghost' bisectors we have to determine where the start */
	for(auto b : {&abBis,&bcBis}) {
		if(b->isGhost()) {
			auto n = nodes[pathFinder[b->eIdxA][1]];
			b->newSource(n.point);
			LOG(INFO) << "ghost bis, set new sourcepoint: " << n.point;
		}
	}
	LOG(INFO) << "bisectors after correction";
	LOG(INFO) << abBis;
	LOG(INFO) << bcBis;


	/* compute bisector intersection, this is the collapse
	 * time of the middle edge (b) 'edge-event' for b  */
	auto intersection = intersectElements(abBis.supporting_line(), bcBis.supporting_line());

	Line b(data.getEdge(bIdx).supporting_line());
	if(intersection != INFPOINT && b.has_on_positive_side(intersection)) {
		auto distance = normalDistance(b, intersection) / (data.w(bIdx)*data.w(bIdx));
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
	LOG(INFO) << e << " ret event. ";

	return e;
}

Bisector Wavefront::constructBisector(const uint& aIdx, const uint& bIdx) const {
	assert(aIdx != bIdx);
	Line a(data.getEdge(aIdx).supporting_line());
	Line b(data.getEdge(bIdx).supporting_line());

	Point intersectionA = intersectElements(a, b);

	LOG(INFO) << " bis " << aIdx << "/" << bIdx << "  ";

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

			if(pBis == intersectionA) {
				bis = Ray(pBis,bisLine.direction());
			}

			if( !a.has_on_positive_side(pBis) || !b.has_on_positive_side(pBis) ) {
				bis = bis.opposite();
			}

			if(bis.is_degenerate() || bis.is_horizontal() || bis.is_vertical()) {
				/* interesting BUG? If one of these conditions is true a Ray leads to
				 * some side effects but a line still works... */
				return Bisector(bis.supporting_line(),aIdx,bIdx);
			}

			return Bisector(bis,aIdx,bIdx);
		} else {
			if(CGAL::collinear(a.point(0),a.point(1),b.point(0)+b.to_vector())) {
				LOG(INFO) << "constructBisector: ghost arc";

				auto bis = Bisector(Line(a.point(0),a.perpendicular(a.point(0)).to_vector()),aIdx,bIdx);

				if(pathFinder[aIdx][1] != MAX) {
					auto n = nodes[pathFinder[aIdx][1]];
					bis.newSource(n.point);
				}

				bis.setGhost(true);

				auto dir = a.perpendicular(a.point(0)).direction();
				if(dir == data.monotonicityLine.direction()) {
					LOG(INFO) << "-- unset ghost vertex";
					bis.setGhost(false);
				}

				return bis;
			} else {
				LOG(INFO) << "constructBisector: bisector of parallel input edges";
				Line bisLine = CGAL::bisector(a,b.opposite());
				auto bis = Bisector(bisLine,aIdx,bIdx);

				if(a.direction() == data.perpMonotonDir || a.opposite().direction() == data.perpMonotonDir) {
					LOG(INFO) << "bisector perpendicular to monotonicity line";
					bis.setParallel(true);
				}

				LOG(INFO) << bisLine;
				return bis;
			}
		}
	} else {
		/* weighted bisector */
		LOG(WARNING) << "weighted bisector is not working in general (but might if envelope property is not violated)!";
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

			Ray bisRay(intersectionA,intersectionB);
			if( !a.has_on_positive_side(bP+bisRay.to_vector()) || !b.has_on_positive_side(bP+bisRay.to_vector()) ) {
				bisRay= bisRay.opposite();
			}

			auto bis = Bisector(bisRay,aIdx,bIdx);
			return bis;

		} else {
			LOG(INFO) << "parallel weighted bisector!";

			if(!CGAL::collinear(a.point(0),a.point(1),b.point(0))) {
				LOG(INFO) << "parallel edges!";

				if(a.direction() == b.direction()) {
					Point A  = a.point(0);
					Point B  = intersectElements(a,b.perpendicular(A));

					Vector v = a.to_vector().perpendicular(CGAL::LEFT_TURN);

					Vector x( (A-B)/(data.w(bIdx)-data.w(aIdx)) );

					auto b = Bisector(Line(A+x,a.direction()),aIdx,bIdx);
					b.setParallel(true);
					return b;
				} else {
					/* wavefront edges move towards each other (possibly) */
					Point A  = a.point(0);
					Line perpL = b.perpendicular(A);
					Point B  = intersectElements(a,perpL);

					Vector lv = perpL.to_vector();
					Vector lvPerp = a.to_vector();

					Point A2 = A + lvPerp + (data.w(aIdx)*lv);
					Point B2 = B + lvPerp - (data.w(bIdx)*lv);

					Line aSk = Line(A,A2);
					Line bSk = Line(B,B2);

					Point pSkInt = intersectElements(aSk,bSk);

					if(pSkInt != INFPOINT) {
						auto b = Bisector(Line(pSkInt,a.direction()),aIdx,bIdx);
						b.setParallel(true);
						return b;
					} else {
						LOG(ERROR) << "skewed lines do not intersect?";
						assert(false);
					}
				}

			} else {
				LOG(ERROR) << "collinear edges!?! -- is not handled as we do not support weights in gerneral";

				auto eA = data.e(aIdx);
				auto eB = data.e(bIdx);

				for(auto i : {eA[0],eA[1]}) {
					for(auto j : {eB[0],eB[1]}) {
						if(i == j) {
							Point P = data.v(i);
							Bisector b(Ray(P,a.direction()),aIdx,bIdx);
							return b;
						}
					}
				}

				assert(false);
				return Bisector(Ray(),aIdx,bIdx);
			}
		}
	}

	assert(false);
	return Bisector(Ray(),aIdx,bIdx);
}

Bisector Wavefront::getBisectorWRTMonotonicityLine(const Bisector& bisector) const {
	Bisector bis(bisector);

	Line lp(ORIGIN,data.perpMonotonDir);
	Point p(ORIGIN+bis.to_vector());

	if(lp.has_on_positive_side(p)) {
		bis.changeDirection();
	} else if(lp.has_on_boundary(p)) {
		bis.setParallel(true);
		LOG(WARNING) << "WARNUNG: ... bisector is perpendicular to monotonicity line.";
	}

	return bis;
}

void Wavefront::ChainDecomposition() {
	Chain lc, uc;

	uint minVertex = data.bbox.monotoneMinIdx;
	uint maxVertex = data.bbox.monotoneMaxIdx;

	LOG(INFO) << "min/max vertex: " << minVertex << " - " << maxVertex;

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
	std::stringstream ss;
	ss << "upper chain: ";
	for(auto e : upperChain) {
		ss << e << " ";
	}
	ss << std::endl << "lower chain: ";
	for(auto e : lowerChain) {
		ss << e << " ";
	}
	LOG(INFO) << ss.str();
}

uint Wavefront::addArcRay(const uint& nodeAIdx, const uint& edgeLeft, const uint& edgeRight, const Ray& ray, const bool vertical, const bool horizontal) {
	auto nodeA = &nodes[nodeAIdx];
	Arc arc(ArcType::RAY, nodeAIdx, edgeLeft, edgeRight, ray,vertical,horizontal);
	auto arcIdx = arcList.size();
	arcList.push_back(arc);
	nodeA->arcs.push_back(arcIdx);

	return arcIdx;
}

uint Wavefront::addArc(const uint& nodeAIdx, const uint& nodeBIdx, const uint& edgeLeft, const uint& edgeRight, const bool vertical, const bool horizontal) {
	auto nodeA = &nodes[nodeAIdx];
	auto nodeB = &nodes[nodeBIdx];
	Arc arc(ArcType::NORMAL, nodeAIdx, nodeBIdx, edgeLeft, edgeRight, Edge(nodeA->point, nodeB->point),vertical,horizontal);
	auto arcIdx = arcList.size();
	arcList.push_back(arc);
	nodeA->arcs.push_back(arcIdx);
	nodeB->arcs.push_back(arcIdx);
	LOG(INFO) << "++ adding arc: " << arc;
	return arcIdx;
}

/* as the polygon boundary is index from 0 to n and we have start and end indices stored
 * we can decide in O(1) if an index in in the lower chain */
bool Wavefront::isEdgeOnLowerChain(const uint edgeIdx) const {
	if(startLowerEdgeIdx < endLowerEdgeIdx) {
		return edgeIdx >= startLowerEdgeIdx && edgeIdx <= endLowerEdgeIdx;
	} else {
		return edgeIdx >= startLowerEdgeIdx || edgeIdx <= endLowerEdgeIdx;
	}
}

void Wavefront::addNewNodefromEvent(const Event& event, PartialSkeleton& skeleton) {
	uint nodeIdx = nodes.size();
	auto paths 	 = pathFinder[event.mainEdge];

	Point Pa = getNode(paths[0])->point, Pb = getNode(paths[1])->point;
	Edge e(Pa,Pb);

	/* if this is already done, i.e., left and/or right path ends at a node of the event */
	if(nodes[paths[0]].point == event.eventPoint && nodes[paths[1]].point == event.eventPoint) {
		nodeIdx = paths[0];
	} else if(nodes[paths[0]].point == event.eventPoint) {
		/* so we use the left referenced node and only create a new arc for the right side */
		nodeIdx = paths[0];
		addArc(paths[1],nodeIdx,event.mainEdge,event.rightEdge,e.is_vertical(),e.is_horizontal());
	} else if(nodes[paths[1]].point == event.eventPoint) {
		/* so we use the left referenced node and only create a new arc for the left side */
		nodeIdx = paths[1];
		addArc(paths[0],nodeIdx,event.leftEdge,event.mainEdge,e.is_vertical(),e.is_horizontal());
	} else {
		/* a classical event to be handled */
		LOG(INFO) << "event point before adding node " << event.eventPoint;
		auto node = Node(NodeType::NORMAL,event.eventPoint,event.eventTime);
		nodes.push_back(node);
		skeleton.push_back(nodeIdx);

		Edge e1(Pa,event.eventPoint);
		Edge e2(Pb,event.eventPoint);
		addArc(paths[0],nodeIdx,event.leftEdge,event.mainEdge,e1.is_vertical(),e1.is_horizontal());
		addArc(paths[1],nodeIdx,event.mainEdge,event.rightEdge,e2.is_vertical(),e2.is_horizontal());
	}

	/* update path finder for left and right edge */
	pathFinder[event.leftEdge][1]  = nodeIdx;
	pathFinder[event.rightEdge][0] = nodeIdx;
}

/**
 * traverse a path that starts on one terminal node of an input edge
 * this traverse occurs in a monotone (i.r.t. the monotonicity line) way
 * */
bool Wavefront::nextMonotoneArcOfPath(MonotonePathTraversal& path) {
	LOG(INFO) << "nextMonotoneArcOfPath" << path;

	if(path.done()) {return false;}

	auto currentArc  = &arcList[path.currentArcIdx];
	auto oppositeArc = &arcList[path.oppositeArcIdx];

	/* check if rightmost end-point of both arcs is the same */
	if(currentArc->adjacent(*oppositeArc)) { // && getRightmostNodeIdxOfArc(currentArc) == getRightmostNodeIdxOfArc(oppositeArc))  {
		path.currentArcIdx = path.oppositeArcIdx;
		LOG(INFO) << "current and opposite are adjacent";
		return true;
	} else if(isArcLeftOfArc(*oppositeArc,*currentArc)) { // && getLeftmostNodeIdxOfArc(currentArc) != getLeftmostNodeIdxOfArc(oppositeArc) ) {
		/* opposite arcs left endpoint is to the left of the current arc ones */
		path.swap();
		path.iterateAwayFromEdge = !path.iterateAwayFromEdge;
		LOG(INFO) << "swap" << path;
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
				path.iterateAwayFromEdge = !path.iterateAwayFromEdge;
			}
			LOG(INFO) << "next arc " << path.currentArcIdx << " found";
			return true;
		} else {
			LOG(ERROR) << "No next arc found!";
			return false;
		}
	}
}

uint Wavefront::getCommonNodeIdx(const uint& arcIdxA, const uint& arcIdxB) {
	if(arcIdxA != MAX && arcIdxB != MAX) {
		Arc* arcA = getArc(arcIdxA);
		Arc* arcB = getArc(arcIdxB);
		return arcA->getCommonNodeIdx(*arcB);
	}
	return MAX;
}

void Wavefront::updateArcNewNode(const uint idx, const uint nodeIdx) {
	auto arc = getArc(idx);
	Node* node = getNode(nodeIdx);
	if(arc->isEdge()) {
		if(arc->firstNodeIdx == nodeIdx) {
			arc->edge = Edge(node->point,arc->edge.target());
		} else if(arc->secondNodeIdx == nodeIdx) {
			arc->edge = Edge(arc->edge.source(),node->point);
		}
	} else /* arc is ray */ {
		LOG(WARNING) << "updateArcNewNode: all arcs should be bounded by now!";
		if(arc->firstNodeIdx == nodeIdx) {
			arc->ray = Ray(node->point,arc->ray.direction());
		}
	}
}

uint Wavefront::getNextArcIdx(const MonotonePathTraversal& path, const Arc& arc) const {
	uint rightNodeIdx; // = getRightmostNodeIdxOfArc(arc);
	uint repeat       = 0;

	if(!arc.isRay()) {
		if(path.iterateAwayFromEdge) {
			rightNodeIdx = arc.secondNodeIdx;
		} else {
			rightNodeIdx = arc.firstNodeIdx;
		}
	} else {
		rightNodeIdx = arc.firstNodeIdx;
	}
	auto rightNode = nodes[rightNodeIdx];

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

bool Wavefront::isArcLeftOfArc(const Arc* arcA, const Arc* arcB) const {
	assert(arcA != nullptr || arcB != nullptr);
	if(arcA == nullptr) {
		return false;
	} else if(arcB == nullptr) {
		return true;
	} else {
		return isArcLeftOfArc(data.monotonicityLine,*arcA,*arcB);
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
		LOG(ERROR) << "(R) Traversing a disabled arc/ray! " << arc.firstNodeIdx << " " << arc;
		if(arc.secondNodeIdx == MAX) {
			return arc.firstNodeIdx;
		} else {
			const auto& Nb = nodes[arc.secondNodeIdx];
			return (data.monotoneSmaller(Na.point,Nb.point)) ? arc.secondNodeIdx : arc.firstNodeIdx;
		}
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
		LOG(ERROR) << "(L) Traversing a disabled arc/ray! " << arc.firstNodeIdx << " " << arc;
		if(arc.secondNodeIdx == MAX) {
			return arc.firstNodeIdx;
		} else {
			const auto& Nb = nodes[arc.secondNodeIdx];
			return (data.monotoneSmaller(Na.point,Nb.point)) ? arc.firstNodeIdx : arc.secondNodeIdx;
		}
	}
}

void Wavefront::initPathForEdge(const bool upper, const uint edgeIdx) {
	/* set the upperPath/lowerPath in 'wf' */
	LOG(INFO) << "initPathForEdge " << edgeIdx;
	Node& terminalNode   = (upper) ? getTerminalNodeForVertex(data.e(edgeIdx)[0]) : getTerminalNodeForVertex(data.e(edgeIdx)[1]);

	MonotonePathTraversal path;

	if(!terminalNode.arcs.empty()) {
		uint  initialArcIdx  = terminalNode.arcs.front();
		Arc&  initialArc     = arcList[initialArcIdx];

		auto ie = pathFinder[edgeIdx];
		Node& distantNode   = (upper) ? nodes[ie[0]] : nodes[ie[1]];
		uint  distantArcIdx = getPossibleRayIdx(distantNode,edgeIdx);
		LOG(INFO) << "distantArcIdx arc idx " << distantArcIdx;

		if(distantArcIdx == INFINITY || distantArcIdx == initialArcIdx) {
			path = MonotonePathTraversal(edgeIdx,initialArcIdx,initialArcIdx,upper);
		} else {
			Arc&  distantArc    = arcList[distantArcIdx];
			if(isArcLeftOfArc(initialArc,distantArc)) {
				path =  MonotonePathTraversal(edgeIdx,initialArcIdx,distantArcIdx,upper);
			} else {
				path = MonotonePathTraversal(edgeIdx,distantArcIdx,initialArcIdx,upper);
				path.iterateAwayFromEdge = false;
			}
		}
	} else {
		path = MonotonePathTraversal(edgeIdx,MAX,MAX,upper);
	}

	if(upper) {
		upperPath = path;
	} else {
		lowerPath = path;
	}

	LOG(INFO) << path;
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
//		std::cout << i << " (" << n.point << ") " << n.arcs.size() << ": ";
//		for(auto a : n.arcs) {
//			std::cout << a << " [";
//			auto arc = &arcList[a];
//			std::cout << arc->firstNodeIdx << "-" << arc->secondNodeIdx << "] ";
//
//			auto nodeB = &nodes[ arc->firstNodeIdx ];
//			if(arc->type != ArcType::RAY && arc->firstNodeIdx == i) {
//				nodeB = &nodes[ arc->secondNodeIdx ];
//			}
//
//			if(arc->firstNodeIdx == i) {
//				std::cout << arc->secondNodeIdx;
//			} else {
//				std::cout << arc->firstNodeIdx;
//			}
//			std::cout <<  " (" << nodeB->arcs.size() << ") ";
//		}
//		std::cout << std::endl;
		/*	-- DEBUG ONLY --*/
		++i;
	}
}

void Wavefront::printChain(const Chain& chain) const {
	std::stringstream ss;
	ss <<  "chain links: ";
	for(auto l : chain) {
		ss << l << " ";
	}
	LOG(INFO) << ss.str();
}

void Wavefront::printEvents() const {
	std::stringstream ss;
	ss << "eventlist: " << std::endl;
	for(auto t : eventTimes) {
		auto e = events[t.edgeIdx];
		ss << e << std::endl;
	}
	LOG(INFO) << ss.str();
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
	data.lines.clear();
	InitializeEventsAndPathsPerEdge();
	InitializeNodes();
}
