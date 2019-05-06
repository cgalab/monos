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
	auto chain     = (lower) ? getLowerChain()	: getUpperChain();
	auto skeleton  = (lower) ? lowerSkeleton 	: upperSkeleton;
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
	auto chain     = (lower) ? getLowerChain()	: getUpperChain();
	auto skeleton  = (lower) ? lowerSkeleton 	: upperSkeleton;

	while(!eventTimes.empty() && !SingleDequeue(chain,skeleton));

	return !eventTimes.empty();
}

bool Wavefront::SingleDequeue(Chain& chain, PartialSkeleton& skeleton) {
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

		return true;
	}
	return false;
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
		eventTimes.erase(item);
	}

	assert(event.mainEdge() == *event.chainEdge);

	events[event.mainEdge()] = Event(event);
	if(event.eventPoint != INFPOINT) {
		auto te = TimeEdge(event.eventTime,event.mainEdge());
		eventTimes.insert(te);
	}
}


Event Wavefront::getEdgeEvent(const uint& aIdx, const uint& bIdx, const uint& cIdx, const ChainRef& it) const {
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
		e = Event(distance,intersection,aIdx,bIdx,cIdx,it);
	} else {
		if(CGAL::collinear(abRay.point(0),abRay.point(1),bcRay.point(1))) {
			LOG(INFO) << "bisectors are collinear!";
			e = Event(0,INFPOINT,aIdx,bIdx,cIdx,it);
		} else {
			e = Event(0,INFPOINT,aIdx,bIdx,cIdx,it);
		}
	}
	return e;
}

Ray Wavefront::constructBisector(const uint& aIdx, const uint& bIdx) const {
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

			if( !a.has_on_positive_side(pBis) ||
					!b.has_on_positive_side(pBis) ) {
				bis = bis.opposite();
			}
			return bis;
		} else {
			Line bisLine = CGAL::bisector(a,b.opposite());
			Point P = intersectElements(bisLine,data.bbox.left);
			if(P == INFPOINT) {
				P = intersectElements(bisLine,data.bbox.top);
			}
			Ray bis(P,bisLine.direction());
			if(data.bbox.outside(P + bisLine.to_vector())) {
				bis = Ray(P,-bisLine.direction());
			}
			return bis;
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

			// DEBUG draw the line in the GUI
//			if(data.gui) {
//			Edge a1 = data.confineRayToBBox( Ray( aOffsetLine.projection(data.getEdge(aIdx).source()) , aOffsetLine.direction() ) );
//			Edge a2 = data.confineRayToBBox( Ray( aOffsetLine.projection(data.getEdge(aIdx).source()) , -aOffsetLine.direction() ) );
//			Edge b1 = data.confineRayToBBox( Ray( bOffsetLine.projection(data.getEdge(bIdx).source()) , bOffsetLine.direction() ) );
//			Edge b2 = data.confineRayToBBox( Ray( bOffsetLine.projection(data.getEdge(bIdx).source()) , -bOffsetLine.direction() ) );
//			data.lines.push_back(a1);
//			data.lines.push_back(a2);
//			data.lines.push_back(b1);
//			data.lines.push_back(b2);
//			}

			return Ray(intersectionA,intersectionB);

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
				return bis;
			} else {
				LOG(ERROR) << "collinear edges -> ghost vertex (TODO)!";
				return Ray();
			}
		}
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
	Node* node = new Node(NodeType::NORMAL,event.eventPoint,event.eventTime);
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
		uint rightNodeIdx = getRightmostNodeIdxOfArc(*currentArc);
		auto rightNode    = nodes[rightNodeIdx];
		uint nextArcIdx   = MAX;
		uint repeat       = 1;

		do{
			for(auto a : rightNode.arcs) {
				if( !path.isAnIndex(a) ) {
					auto arc = arcList[a];
					if( liesOnFace(arc,path.edgeIdx) ) {
						nextArcIdx = a;
						break;
					}
				}
			}
			if(nextArcIdx == MAX) {
				rightNode = nodes[currentArc->getSecondNodeIdx(rightNodeIdx)];
			}
		} while(nextArcIdx == MAX && repeat-- > 0);

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
//		std::cout << std::boolalpha << "acute angels: " << rayAPointsLeft << " " << rayBPointsLeft << std::endl;
//		std::cout << "x of vects: " << arcA.ray.to_vector().x().doubleValue() << "   " << arcB.ray.to_vector().x().doubleValue() << "   ";

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

bool Wavefront::isArcLeftOfArc(const Arc& arcA, const Arc& arcB) const {
	return isArcLeftOfArc(data.monotonicityLine,arcA,arcB);
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

			bool test = isArcLeftOfArc(initialArc,distantArc);
			std::cout << std::boolalpha << "test " << test << std::endl; fflush(stdout);

			path = (isArcLeftOfArc(initialArc,distantArc)) ? MonotonePathTraversal(edgeIdx,initialArcIdx,distantArcIdx) : MonotonePathTraversal(edgeIdx,distantArcIdx,initialArcIdx,upper);
		}
	} else {
		path = MonotonePathTraversal(edgeIdx,MAX,MAX,upper);
	}

	if(upper) {
		upperPath = path;
		std::cout << upperPath << std::endl;
	} else {
		lowerPath = path;
		std::cout << lowerPath << std::endl;
	}

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
