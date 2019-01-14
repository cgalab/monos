#include "Wavefront.h"

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
		Node n(NodeType::TERMINAL, v);
		nodes.push_back(n);
	}
}

bool Wavefront::ComputeSkeleton(bool lower) {
	auto chain     = (lower) ? getLowerChain()	: getUpperChain();
	auto skeleton  = (lower) ? lowerSkeleton 	: upperSkeleton;
	/** compute all finite edge events
	 *  iterate along lower chain and find event time for each edge
	 **/
	if(chain.size() < 2) {
		return true;
	}

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
		auto event = getEdgeEvent(aEdgeIdx,bEdgeIdx,cEdgeIdx);
		if(event.isEvent()) {
			event.chainEdge = ChainRef(it);
			events[event.mainEdge()] = event;
			auto te = TimeEdge(event.eventTime,event.mainEdge());
			eventTimes.insert(te);
		}

		/* iterate over the chainIterator */
		it = chainIterator;
		++chainIterator;
		aEdgeIdx = bEdgeIdx;
		bEdgeIdx = cEdgeIdx;
	} while(chainIterator != chain.end());

	/*********************************************/
	/* compute skeleton by working through queue */
	/*********************************************/

	currentTime = 0;
	while(!eventTimes.empty()) {
		auto etIt  = eventTimes.begin();
		auto event = &events[etIt->edgeIdx];
		auto eventTime = etIt->time;
		eventTimes.erase(etIt);

		if(currentTime <= eventTime) {
			currentTime = eventTime;

			/* build skeleton from event */
			addNewNodefromEvent(*event,skeleton);

			/* check neighbors for new events, and back into the queue */
			/* edges B,C are the two edges left, right of the event edge,
			 * A,D their respective neighbors */
			updateNeighborEdgeEvents(*event,chain);

			/* remove this edge from the chain (wavefront) */
			chain.erase(event->chainEdge);
			disableEdge(event->mainEdge());

		}
	}

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

			/* compute bisector between the two edges */
			auto bis = constructBisector(aEdgeIdx,bEdgeIdx);
			bis = Ray(node->point,bis.direction());
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

	ChainRef it(event.chainEdge);
	--it;

	if(*it == edgeB && edgeB != chain.front()) {
		--it;		edgeA = *it;

		auto neighborEvent = getEdgeEvent(edgeA,edgeB,edgeC);
		it = ChainRef(event.chainEdge);
		neighborEvent.chainEdge = --it;
		updateInsertEvent(neighborEvent);
	}

	it = ChainRef(event.chainEdge);
	++it;

	if(*it == edgeC && edgeC != chain.back()) {
		++it;		edgeD = *it;

		auto neighborEvent = getEdgeEvent(edgeB,edgeC,edgeD);
		it = ChainRef(event.chainEdge);
		neighborEvent.chainEdge = ++it;
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
		eventTimes.erase(item);
	}

	assert(event.mainEdge() == *event.chainEdge);

	events[event.mainEdge()] = Event(event);
	if(event.eventPoint != INFPOINT) {
		auto te = TimeEdge(event.eventTime,event.mainEdge());
		eventTimes.insert(te);
	}
}


Event Wavefront::getEdgeEvent(const uint& aIdx, const uint& bIdx, const uint& cIdx) const {
	Event e;

	/* compute bisector from edges */
	auto abRay = constructBisector(aIdx, bIdx);
	abRay = Ray(nodes[pathFinder[aIdx][1]].point,abRay.direction());
	auto bcRay = constructBisector(bIdx, cIdx);
	bcRay = Ray(nodes[pathFinder[bIdx][1]].point,bcRay.direction());

	/* compute bisector intersection, this is the collapse
	 * time of the middle edge (b) 'edge-event' for b  */
	auto intersection = intersectElements(abRay, bcRay);

	if(intersection != INFPOINT) {
		Line b(data.getEdge(bIdx).supporting_line());
		auto distance = normalDistance(b, intersection);
		assert(distance > 0);
		/* does collapse so we create an event
		 * and add it to the queue
		 **/
		e = Event(distance,intersection,aIdx,bIdx,cIdx);
	} else {
		e = Event(0,INFPOINT,aIdx,bIdx,cIdx);
	}
	return e;
}

Ray Wavefront::constructBisector(const uint& aIdx, const uint& bIdx) const {
	Line a(data.getEdge(aIdx).supporting_line());
	Line b(data.getEdge(bIdx).supporting_line());

	assert(!isLinesParallel(a,b));

	Point intersectionA = intersectElements(a, b);

	if( data.w(aIdx) == 1 && data.w(bIdx) == 1) {
		Point aP, bP, cP;
		aP = data.eA(aIdx);
		bP = intersectionA;
		cP = data.eB(bIdx);

		Line bisLine = CGAL::bisector(a,b.opposite());
		Point pBis = bisLine.point();
		Ray bis(intersectionA,pBis);

		if( !a.has_on_positive_side(pBis) ||
		    !b.has_on_positive_side(pBis) ) {
			bis = bis.opposite();
		}
		return bis;
	} else {
		LOG(WARNING) << "weighted bisectors not implemented!";
		auto aOffsetLine = getWeightedOffsetLine(aIdx);
		auto bOffsetLine = getWeightedOffsetLine(bIdx);
		Point intersectionB = intersectElements(aOffsetLine, bOffsetLine);

		return Ray(intersectionA,intersectionB);
	}
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
	Node* node = new Node(NodeType::NORMAL,event.eventPoint);
	uint nodeIdx = nodes.size();
	auto paths 	 = pathFinder[event.mainEdge()];

	/* if this is already done, i.e., left and/or right path ends at a node of the event */
	if(nodes[paths[0]].point == event.eventPoint && nodes[paths[1]].point == event.eventPoint) {
		nodeIdx = paths[0];
	} else if(nodes[paths[0]].point == event.eventPoint) {
		/* so we use the left referenced node and only create a new arc for the right side */
		node    = &nodes[paths[0]];
		nodeIdx = paths[0];
		addArc(paths[1],nodeIdx,event.mainEdge(),event.rightEdge());
	} else if(nodes[paths[1]].point == event.eventPoint) {
		/* so we use the left referenced node and only create a new arc for the left side */
		node    = &nodes[paths[1]];
		nodeIdx = paths[1];
		addArc(paths[0],nodeIdx,event.leftEdge(),event.mainEdge());
	} else {
		/* a classical event to be handled */
		nodes.push_back(*node);
		skeleton.push_back(nodeIdx);

		addArc(paths[0],nodeIdx,event.leftEdge(),event.mainEdge());
		addArc(paths[1],nodeIdx,event.mainEdge(),event.rightEdge());
	}

	/* update path finder for left and right edge */
	pathFinder[event.leftEdge()][1]  = nodeIdx;
	pathFinder[event.rightEdge()][0] = nodeIdx;
}

//Transformation rotate(CGAL::ROTATION, sin(pi), cos(pi));
//Transformation rational_rotate(CGAL::ROTATION,Direction(1,1), 1, 100);
Line Wavefront::getWeightedOffsetLine(const uint& i) const {
	auto line = data.getEdge(i).supporting_line();
	auto lineVector = line.to_vector();
	auto lineVectorPer = Vector(-lineVector.y(),lineVector.x());

	auto vectorLen = lineVectorPer.squared_length();
	auto lineVectPerNorm = lineVectorPer/vectorLen;
	auto aPerpDirWeighted = lineVectPerNorm * data.w(i);

	Transformation translate(CGAL::TRANSLATION, aPerpDirWeighted );
	return line.transform(translate);
}


/* used for the output, a arc may loses its index in its firstNode when the merge is done */
bool Wavefront::isArcInSkeleton(const uint& arcIdx) const {
	auto arc = &arcList[arcIdx];
	auto node = &nodes[arc->firstNodeIdx];
	uint appearancCnt = 0;
	for(uint i = 0; i < 2; ++i) {
		for(auto idx : node->arcs) {
			if(idx == arcIdx) {
				++appearancCnt;
			}
		}
		node = &nodes[arc->secondNodeIdx];
	}
	return appearancCnt == 2;
}

void Wavefront::SortArcsOnNodes() {
	uint i = 0;
	for(auto n : nodes) {
		n.sort(arcList);
		std::cout << i << " (" << n.point << ") " << n.arcs.size() << ": ";
		for(auto a : n.arcs) {
			std::cout << a << " [";
			auto arc = &arcList[a];
			std::cout << arc->firstNodeIdx << "-" << arc->secondNodeIdx << "] ";
			auto nodeB = &nodes[ (arc->firstNodeIdx == i) ? arc->secondNodeIdx : arc->firstNodeIdx  ];
			if(arc->firstNodeIdx == i) {
				std::cout << arc->secondNodeIdx;
			} else {
				std::cout << arc->firstNodeIdx;
			}
			std::cout <<  " (" << nodeB->arcs.size() << ") ";
		}
		std::cout << std::endl;
		++i;
	}
}

