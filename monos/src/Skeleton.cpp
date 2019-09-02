
#include "Skeleton.h"

std::ostream& operator<< (std::ostream& os, const Intersection& intersection) {
	os << "intersection point: " << intersection.getIntersection() << std::endl << " arcs: ";
	for(auto a : intersection.getArcs()) {
		os << a << " ";
	}
	return os;
}

uint Skeleton::nextUpperChainIndex(const uint& idx) const {
	return (idx > 0) ? idx - 1 : data.getPolygon().size() - 1;
}

uint Skeleton::nextLowerChainIndex(const uint& idx) const {
	return (idx+1 < data.getPolygon().size()) ? idx + 1 : 0;
}

void Skeleton::initMerge() {
	upperChainIndex    	= wf.endUpperEdgeIdx;
	lowerChainIndex    	= wf.startLowerEdgeIdx;
	startIdxMergeNodes 	= wf.nodes.size();
	sourceNodeIdx 	   	= wf.pathFinder[lowerChainIndex][0];
	sourceNode    		= &wf.nodes[sourceNodeIdx];
	newNodeIdx    		= sourceNodeIdx;

	/* set the upperPath/lowerPath in 'wf' */
	wf.initPathForEdge(true,upperChainIndex);
	wf.initPathForEdge(false,lowerChainIndex);
}

/* add last are, connect to end node! */
void Skeleton::finishMerge() {
	Edge e(wf.getNode(mergeEndNodeIdx())->point, wf.getNode(sourceNodeIdx)->point);
	wf.addArc(mergeEndNodeIdx(),sourceNodeIdx,lowerChainIndex,upperChainIndex,e.is_vertical(),e.is_horizontal());
	computationFinished = true;
	LOG(INFO) << "Merge Finished!";
}

void Skeleton::MergeUpperLowerSkeleton() {
	initMerge();

	/* iterate over upper and lower chain with while loop */
	while(SingleMergeStep());

	finishMerge();
}


/**
 * Executes a single merge step along the merge line, return false if merge is finished
 * */
bool Skeleton::SingleMergeStep() {
	LOG(INFO) << "################################### START SINGLE MERGE STEP " << upperChainIndex << "/" << lowerChainIndex << " ######################";


	/* we start the bisector from the source node from the "left" since the merge line is monotone */
	Bisector bisGeneral = wf.constructBisector(upperChainIndex,lowerChainIndex);
	Bisector bis = wf.getBisectorWRTMonotonicityLine(bisGeneral);
	bis.newSource(sourceNode->point);

	LOG(INFO) << "Bisector-dir: " << bis.direction();

	/* visualize next bisector via dashed line-segment */
	data.visualizeBisector(Edge(sourceNode->point,sourceNode->point+(5*bis.to_vector())));

	/* setup intersection call */
	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	IntersectionPair intersectionPair = findNextIntersectingArc(bis);
	LOG(INFO) << "intersection upper: " << intersectionPair.first << std::endl << "intersection lower: " << intersectionPair.second;

	/**
	 * IF we need to handle a source ghost node from the previous round!
	 * This means that the next arc is a ghost arc that will end at that ghost node,
	 * therefore its position is not yet fixed and we have to move it 'now'. */
	handleSourceGhostNode(bis,intersectionPair);

	/**
	 * we distinguish between 'simple' intersections, meaning an intersection point is left of the other
	 * or there is only one, and the rest where node intersections and things like that may occur */
	if(isVerticalIntersectionButSimple(bis,intersectionPair) || isIntersectionSimple(intersectionPair)) {
		bool onUpperChain;
		auto intersection = getIntersectionIfSimple(bis,intersectionPair,onUpperChain);
		LOG(INFO) << "-- handleMerge upper:" << onUpperChain << ", point: " << intersection;
		newNodeIdx = handleMerge(intersection,upperChainIndex,lowerChainIndex,bis);
		Arc* modifiedArc = &wf.arcList[*intersection.getArcs().rbegin()];
		if(onUpperChain) {
			upperChainIndex = (modifiedArc->leftEdgeIdx != upperChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			wf.initPathForEdge(true,upperChainIndex);
		} else {
			lowerChainIndex = (modifiedArc->leftEdgeIdx != lowerChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			wf.initPathForEdge(false,lowerChainIndex);
		}
	} else {
		/* more than two arcs means at least one existing node lies on the intersection */
		LOG(INFO) << "-- handleDoubleMerge!";
		newNodeIdx = handleDoubleMerge(intersectionPair,upperChainIndex,lowerChainIndex,bis);

		Point Pu = data.eA(wf.startLowerEdgeIdx);
		Point Pl = data.eB(wf.startLowerEdgeIdx);
		uint lowerArc = MAX, upperArc = MAX;

		for(auto aIdx : intersectionPair.first.getArcs()) {
			auto arcIt = wf.getArc(aIdx);
			/* arc of upper skeleton */
			Point PCheck = data.eA(arcIt->leftEdgeIdx);
			if(Pu == PCheck || data.monotoneSmaller(Pu,PCheck)) {
				upperArc = aIdx;
				Pu = PCheck;
			}
		}

		for(auto aIdx : intersectionPair.second.getArcs()) {
			auto arcIt = wf.getArc(aIdx);
			/* arc of lower skeleton */
			Point PCheck = data.eB(arcIt->rightEdgeIdx);
			if(Pl == PCheck || data.monotoneSmaller(Pl,PCheck)) {
				lowerArc = aIdx;
				Pl = PCheck;
			}
		}

		if(upperArc != MAX) {
			upperChainIndex = wf.getArc(upperArc)->leftEdgeIdx;
			wf.initPathForEdge(true,upperChainIndex);
		}

		if(lowerArc != MAX) {
			lowerChainIndex = wf.getArc(lowerArc)->rightEdgeIdx;
			wf.initPathForEdge(false,lowerChainIndex);
		}
	}

//	/* the new node may reside on an arc that is not in the current focus */
//	checkAndRepairCollinearArcs();

	/* we iterate the sourcenode to the newly added node and go on merging */
	LOG(INFO) << "changing idx from old: " << sourceNodeIdx << " to " << newNodeIdx;
	sourceNodeIdx = newNodeIdx;
	sourceNode = &wf.nodes[sourceNodeIdx];

	/* addGhostNode is set to true on certain conditions to handle a ghost node in the next
	 * merge round */
	if(addGhostNode) {
		sourceNode->setGhost(true);
		addGhostNode = false;
		LOG(INFO) << "(G) SingleMergeStep: setting sourceNode to be a ghost node!";
	}

	LOG(INFO) << "";
	LOG(INFO) << "############################################## END STEP ###########################";
	LOG(INFO) << "";

	return !EndOfBothChains();
}

/* finds the next arc(s) intersected by the bisector 'bis' that lie closest to 'sourceNode'
 * in respect to the 'monotonicityLine' */
IntersectionPair Skeleton::findNextIntersectingArc(Bisector& bis) {
	assert(sourceNode != nullptr);

	/* holds the arcs we find in this search */
	Intersection upperIntersection, lowerIntersection;

	wf.initPathForEdge(true,upperChainIndex);
	wf.initPathForEdge(false,lowerChainIndex);

	/* new intersection point must be to the right of 'currentPoint' in respect to the monotonicity line */
	bool localOnUpperChain;

	MonotonePathTraversal pathBackupLower, pathBackupUpper;

	MonotonePathTraversal *path;
	Arc *arc, *arc_u, *arc_l;
	Intersection *intersection;

	if(bis.isParallel() && bis.isRay()) {bis.changeToLine(); LOG(INFO) << "change bis-ray to line.";}

	/* while iterate we may iterate one arc to far, this is an easy way to step back */
	pathBackupLower = wf.lowerPath; pathBackupUpper = wf.upperPath;

	LOG(INFO) << "findNextIntersectingArc start "  << upperIntersection.isDone() << ", " << lowerIntersection.isDone();
	arc_u = (EndOfUpperChain()) ? nullptr : wf.getArc(wf.upperPath);
	arc_l = (EndOfLowerChain()) ? nullptr : wf.getArc(wf.lowerPath);

	if(arc_u == nullptr) {upperIntersection.setDone();}
	if(arc_l == nullptr) {lowerIntersection.setDone();}

	while( !EndOfBothChains() && ( !upperIntersection.isDone() || !lowerIntersection.isDone() ) ) {
		/* check which arc lies further to the left */
		LOG(INFO) << "arc check, arc_u " << *arc_u << ", arc_l: " << *arc_l;
		localOnUpperChain = wf.isArcLeftOfArc(arc_u,arc_l); // && !upperIntersection.isDone();

		if( localOnUpperChain && upperIntersection.isDone()) {localOnUpperChain = false;}
		if(!localOnUpperChain && lowerIntersection.isDone()) {localOnUpperChain = true; }

		path 		 = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
		arc  		 = (localOnUpperChain) ? arc_u         : arc_l;
		intersection = (localOnUpperChain) ? &upperIntersection : &lowerIntersection;

		LOG(INFO) << std::boolalpha << "# localOnUpperChain: " << localOnUpperChain << " upper: "<<upperIntersection.isDone()<< ", lower: " << lowerIntersection.isDone();;
		LOG(INFO) << "# BEFORE path: " << *path;
		LOG(INFO) << "# intersect arc: " << *arc << ", and bisector " << bis;


		if(handleGhostVertex(*path,bis,*intersection)) {
			/* detect and handle possible ghost vertex */
			/* all good, done in 'handleGhostVertex' */
			LOG(INFO) << "(if) handleGhostVertex";
		} else if(isValidArc(path->currentArcIdx)) {
			/* classical intersection detection on current paths arc */
			Point P = INFPOINT;

			/* handle special intersection cases first (or they freeze the intersection function?) */
			if(isNodeIntersectionAndVerticalBisector(bis,arc->firstNodeIdx)) {
				P = wf.getNode(arc->firstNodeIdx)->point;
			} else if(arc->isEdge() && isNodeIntersectionAndVerticalBisector(bis,arc->secondNodeIdx)) {
				P = wf.getNode(arc->secondNodeIdx)->point;
			} else {
				P = intersectBisectorArc(bis,*arc);
			}

			/* in case of a ghost-bisector we have to find the path-path-intersection */
			if(bis.isGhost() && sourceNode->isGhostNode()) {
				P = intersectArcArc(*arc_u,*arc_l);
				if(P != INFPOINT) {
					LOG(INFO) << "isGhost found intersection: " << P;
					upperIntersection.add(P,wf.upperPath.currentArcIdx);
					lowerIntersection.add(P,wf.lowerPath.currentArcIdx);
					upperIntersection.setDone();
					lowerIntersection.setDone();
				}
			}

			if(P != INFPOINT) {
				if(!intersection->isDone()) {
					intersection->add(P,path->currentArcIdx);
					checkNodeIntersection(*intersection,arc);
					reevaluateIntersectionIfMultipleArcs(bis, *intersection);
					intersection->setDone();
				}
			} else {
				LOG(INFO) << "## TEST THIS when P = INFPOINT!?!";

				/* while iterate we may iterate one arc to far, this is an easy way to step back */
				if(localOnUpperChain) {	pathBackupUpper = wf.upperPath;
				} else {				pathBackupLower = wf.lowerPath;}

				if(!wf.nextMonotoneArcOfPath(*path)) {
					LOG(INFO) << "OK";
					Edge e = data.getEdge(path->edgeIdx);
					if(do_intersect(bis,e)) {
						LOG(INFO) << "intersecting input edge (done)";
						intersection->setDone();
					} else 	{
						/* iterate over path */ LOG(WARNING) << "///////////////// next chain index! ";
						intersection->setDone();
						LOG(INFO) << "I do not think we should iterate to the next face just like that!";
					}
				} else {
					LOG(INFO) << "OK(2)";
					if(localOnUpperChain) {
						arc_u = (EndOfUpperChain()) ? nullptr : wf.getArc(wf.upperPath);
					} else {
						arc_l = (EndOfLowerChain()) ? nullptr : wf.getArc(wf.lowerPath);
					}
				}
			}
		}
	}

	LOG(INFO) << std::boolalpha << "AFTER success: " << intersection->isDone() << ", path: " << *path;
	LOG(INFO) << "findNextIntersectingArc END";

	IntersectionPair intersectionPair = std::make_pair(upperIntersection,lowerIntersection);

	multiEventCheck(bis,intersectionPair);
	if(!sourceNode->isGhostNode() || (sourceNode->isGhostNode() && sourceNode->degree() > 2) ) {
		if(!data.isEdgeCollinearAndInteriorRight(upperChainIndex,lowerChainIndex)) {
			checkAndAddCollinearArcs(intersectionPair);
		}
	}

	return intersectionPair;
}

void Skeleton::multiEventCheck(const Bisector& bis, IntersectionPair& pair) {
	if(!pair.first.empty() && !pair.second.empty() && !data.isEdgesParallel(upperChainIndex,lowerChainIndex)) {
		std::set<uint> upperEdges, lowerEdges;
		for(auto a : pair.first.getArcs()) {
			auto arc = wf.getArc(a);
			upperEdges.insert(arc->leftEdgeIdx);
			upperEdges.insert(arc->rightEdgeIdx);
		}
		for(auto a : pair.second.getArcs()) {
			auto arc = wf.getArc(a);
			lowerEdges.insert(arc->leftEdgeIdx);
			lowerEdges.insert(arc->rightEdgeIdx);
		}

		LOG(INFO) << "going";
		Point P = pair.first.getIntersection();
		Exact distU = -CORE_ONE, distL = -CORE_ONE;
		LOG(INFO) << "distL: " << distL.doubleValue() << ", distU: " << distU.doubleValue();

		bool done = false;

		for(auto i : upperEdges) {
			for(auto j : lowerEdges) {
				LOG(INFO) << "checking collinearity of edge " << i << " and " << j;
				if(i != j && data.isEdgeCollinearAndCommonInteriorDirection(i,j)) {
					done = true;
				}
			}
		}

		if(!done) {
			for(auto i : upperEdges) {
				if(i != bis.eIdxA && i != bis.eIdxB && i != upperChainIndex) {
					auto l = data.getEdge(i).supporting_line();
					distU = normalDistance(l,P);
					break;
				}
			}
			for(auto i : lowerEdges) {
				if(i != bis.eIdxA && i != bis.eIdxB && i != lowerChainIndex) {
					auto l = data.getEdge(i).supporting_line();
					distL = normalDistance(l,P);
					break;
				}
			}
		}


		LOG(INFO) << "distL: " << distL.doubleValue() << ", distU: " << distU.doubleValue();
		if(done || (distL != -CORE_ONE && distL == distU)) {
			LOG(INFO) << "setting intersections";
			pair.first.setIntersection(P);
			pair.second.setIntersection(P);
		}
	}
}

void Skeleton::checkAndAddCollinearArcs(IntersectionPair& pair) {
	Point intersection = pair.first.getIntersection();
	if(intersection == INFPOINT) {
		intersection = pair.second.getIntersection();
	}
	assert(intersection != INFPOINT);

	for(auto arcIdx : sourceNode->arcs) {
		auto arc = wf.getArc(arcIdx);
	LOG(INFO) << "arc: " << *arc;
		if(arc->has_on(intersection)) {
			if(!wf.isEdgeOnLowerChain(arc->leftEdgeIdx)) {
				LOG(INFO) << "checkAndAddCollinearArcs (upper): " << arcIdx;
				pair.first.add(intersection,arcIdx);
				upperChainIndex = arc->rightEdgeIdx;
			} else /* lower chain */ {
				LOG(INFO) << "checkAndAddCollinearArcs (lower): " << arcIdx;
				pair.second.add(intersection,arcIdx);
				lowerChainIndex = arc->leftEdgeIdx;
			}
		}
	}
}

bool Skeleton::isNodeIntersectionAndVerticalBisector(const Bisector& bis, const uint nodeIdx) const {
	auto node = wf.getNode(nodeIdx);
	std::set<uint> edges;

	if(bis.is_vertical()) {
		for(auto a : node->arcs) {
			auto arc = wf.getArc(a);
			edges.insert(arc->leftEdgeIdx);
			edges.insert(arc->rightEdgeIdx);
		}

		bool foundLeft = false, foundRight = false;

		for(auto eIdx : edges) {
			if(eIdx == bis.eIdxA || data.isEdgeCollinear(eIdx,bis.eIdxA)) {
				foundLeft = true;
			}
			if(eIdx == bis.eIdxB || data.isEdgeCollinear(eIdx,bis.eIdxB)) {
				foundRight = true;
			}
		}

		if(foundLeft && foundRight) {
			return true;
		}
	}

	return false;
}

void Skeleton::reevaluateIntersectionIfMultipleArcs(const Bisector& bis, Intersection& intersection) {
	if(bis.isAA()) {
		LOG(INFO) << "reevaluateIntersectionIfMultipleArcs";
		if(intersection.size() > 1) {
			Point Pref = sourceNode->point;
			Exact dist = CGAL::abs(Pref.y() - intersection.getIntersection().y());
			for(auto aIdx : intersection.getArcs()) {
				auto arc = wf.getArc(aIdx);
				if(arc->isEdge() && arc->is_vertical()) {
					auto nodeA = wf.getNode(arc->firstNodeIdx);
					auto nodeB = wf.getNode(arc->secondNodeIdx);
					Exact distComp;
					if(intersection.getIntersection().y() != nodeA->point.y()) {
						distComp = CGAL::abs(Pref.y() - nodeA->point.y());
						if(distComp < dist) {
							dist = distComp;
							intersection.clear();
							intersection.add(nodeA->point,aIdx);
						}
					}
					if(intersection.getIntersection().y() != nodeB->point.y()) {
						distComp = CGAL::abs(Pref.y() - nodeB->point.y());
						if(distComp < dist) {
							dist = distComp;
							intersection.clear();
							intersection.add(nodeB->point,aIdx);
						}
					}
				}
			}
		}
	}
}

void Skeleton::checkNodeIntersection(Intersection& intersection, const Arc* arc) {
	uint runs = (arc->isEdge()) ? 2 : 1;
	uint nodeIdx = arc->firstNodeIdx;

	while(runs-- > 0) {
		auto node = wf.getNode(nodeIdx);
		if(intersection.getIntersection() == node->point) {
			for(auto aIdx : node->arcs) {
				auto arcIt = wf.getArc(aIdx);
				if(arcIt->firstNodeIdx == nodeIdx || arcIt->secondNodeIdx == nodeIdx) {
					intersection.addArc(aIdx);
				}
			}
		}

		nodeIdx = arc->secondNodeIdx;
	}
}

bool Skeleton::isIntersectionSimple(const IntersectionPair& pair) const {
	Point Pa = pair.first.getIntersection();
	Point Pb = pair.second.getIntersection();
	Point PaMON = data.pointOnMonotonicityLine(Pa);
	Point PbMON = data.pointOnMonotonicityLine(Pb);
	return PaMON != PbMON;
}

bool Skeleton::isVerticalIntersectionButSimple(const Bisector& bis, const IntersectionPair& pair) const {
	if(bis.is_vertical()) {
		Point Pa = pair.first.getIntersection();
		Point Pb = pair.second.getIntersection();
		return Pa.y() != Pb.y();
	}
	return false;
}

Intersection Skeleton::getIntersectionIfSimple(const Bisector& bis, const IntersectionPair& pair, bool& onUpperChain) const {
	Point Pa = pair.first.getIntersection();
	Point Pb = pair.second.getIntersection();

	if(pair.first.empty()) {
		assert(!pair.second.empty());
		onUpperChain = false;
		return pair.second;
	} else if(pair.second.empty()) {
		assert(!pair.first.empty());
		onUpperChain = true;
		return pair.first;
	}

	if(bis.is_vertical()) {
		LOG(INFO) << "getIntersectionIfSimple";
		Exact distA = CGAL::abs(sourceNode->point.y() - Pa.y());
		Exact distB = CGAL::abs(sourceNode->point.y() - Pb.y());
		if( distA < distB ) {
			onUpperChain = true;
			return pair.first;
		} else {
			onUpperChain = false;
			return pair.second;
		}
	}

	if( data.monotoneSmaller(data.pointOnMonotonicityLine(Pa),data.pointOnMonotonicityLine(Pb) ) ) {
		onUpperChain = true;
		return pair.first;
	} else {
		onUpperChain = false;
		return pair.second;
	}
}

void Skeleton::initNextChainAndPath(bool upperChain) {
	if(upperChain) {
		upperChainIndex = nextUpperChainIndex(upperChainIndex);
		wf.initPathForEdge(true,upperChainIndex);
	} else {
		lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
		wf.initPathForEdge(false,lowerChainIndex);
	}
}

uint Skeleton::handleDoubleMerge(IntersectionPair& intersectionPair, const uint& edgeIdxA, const uint& edgeIdxB, const Bisector& bis) {
	uint newNodeIdx = MAX;
	/* if more than two arcs are involved in one 'Intersection' we have an existing node
	 * to connect to */

	auto arcs = intersectionPair.first.getArcs();

	if(intersectionPair.first.size() > 1 || intersectionPair.second.size() > 1) {
		LOG(INFO) << "handleDoubleMerge: intersecting a node";
		uint upperNodeIdx = MAX, lowerNodeIdx = MAX;
		upperNodeIdx = wf.getCommonNodeIdx(intersectionPair.first.getFirstArcIdx(),intersectionPair.first.getSecondArcIdx());
		lowerNodeIdx = wf.getCommonNodeIdx(intersectionPair.second.getFirstArcIdx(),intersectionPair.second.getSecondArcIdx());

		if(upperNodeIdx != MAX && lowerNodeIdx != MAX) {
			newNodeIdx = upperNodeIdx;
		} else if(upperNodeIdx != MAX) {
			newNodeIdx = upperNodeIdx;
		} else if(lowerNodeIdx != MAX){
			newNodeIdx = lowerNodeIdx;
		} else {
			assert(false);
		}

		wf.addArc(sourceNodeIdx,newNodeIdx,edgeIdxA,edgeIdxB,bis.is_vertical(),bis.is_horizontal());

		for(auto arcIdx : arcs) {
			if(arcIdx < MAX) {
				auto arc = wf.getArc(arcIdx);
				if(upperNodeIdx == arc->firstNodeIdx && !arc->is_vertical()) {
					arc->disable();
				} else {
					/* TODO: parallel-bisectors, direction unclear */
					LOG(INFO) << "update arc target " << arcIdx << " to end at " << newNodeIdx;
					updateArcTarget(arcIdx,edgeIdxA,newNodeIdx,intersectionPair.first.getIntersection());
					LOG(INFO) << "after update";
				}
			}
		}

		arcs = intersectionPair.second.getArcs();

		if( intersectionHasVerticalCoallignedArc(intersectionPair)
				||
			(bis.is_vertical() && intersectionHasVerticalArc(intersectionPair))
		) {
			LOG(INFO) << "<- set addGhostNode true (handleDoubleMerge)";
			addGhostNode = true;
		}

	} else {
		if(bis.is_vertical()) {
			auto upperArc = wf.getArc(intersectionPair.first.getFirstArcIdx());
			auto lowerArc = wf.getArc(intersectionPair.second.getFirstArcIdx());

			if(upperArc->is_vertical() && lowerArc->is_vertical()) {
				newNodeIdx = handleMerge(intersectionPair.first,edgeIdxA,edgeIdxB,bis);
				arcs = intersectionPair.second.getArcs();
				LOG(INFO) << "<- set addGhostNode true (handleDoubleMerge)";
				addGhostNode = true;
			} else if(upperArc->is_vertical()) {
				newNodeIdx = handleMerge(intersectionPair.first,edgeIdxA,edgeIdxB,bis);
				arcs = std::set<uint>();
				intersectionPair.second.clear();
				LOG(INFO) << "<- set addGhostNode true (handleDoubleMerge)";
				addGhostNode = true;
			} else if(lowerArc->is_vertical()) {
				newNodeIdx = handleMerge(intersectionPair.second,edgeIdxA,edgeIdxB,bis);
				arcs = std::set<uint>();
				intersectionPair.first.clear();
				LOG(INFO) << "<- set addGhostNode true (handleDoubleMerge)";
				addGhostNode = true;
			}

		} else if(intersectionPair.first.getIntersection() == intersectionPair.second.getIntersection()) {
			newNodeIdx = handleMerge(intersectionPair.first,edgeIdxA,edgeIdxB,bis);
			arcs = intersectionPair.second.getArcs();
		} else if(data.monotoneSmaller(intersectionPair.first.getIntersection(),intersectionPair.second.getIntersection())) {
			LOG(INFO) << "first";
			newNodeIdx = handleMerge(intersectionPair.first,edgeIdxA,edgeIdxB,bis);
			arcs = std::set<uint>();
		} else {
			LOG(INFO) << "second";
			newNodeIdx = handleMerge(intersectionPair.second,edgeIdxA,edgeIdxB,bis);
			arcs = std::set<uint>();
		}
	}

	auto newNode = wf.getNode(newNodeIdx);

	for(auto arcIdx : arcs) {
		if(arcIdx < MAX) {
			/* TODO: parallel-bisectors, direction unclear */
			LOG(INFO) << "update arc target " << arcIdx << " to end at " << newNodeIdx;
			updateArcTarget(arcIdx,edgeIdxA,newNodeIdx,newNode->point);
			LOG(INFO) << "after update";
		}
	}
	return newNodeIdx;
}

void Skeleton::removeRaysFromIntersection(Intersection& intersection) {
	for(auto arcIdx : intersection.getArcs())  {
		auto arc = wf.getArc(arcIdx);
		if(arc->isRay()) {
			auto node = wf.getNode(arc->firstNodeIdx);
			node->removeArc(arcIdx);
			arc->disable();
		}
	}
}

uint Skeleton::handleMerge(const Intersection& intersection, const uint& edgeIdxA, const uint& edgeIdxB, const Bisector& bis) {
	bool fromAtoB = true;
	Exact distB;

	if(!bis.is_vertical()) {
		distB = data.normalDistance(edgeIdxA,intersection.getIntersection());
		fromAtoB = data.normalDistance(edgeIdxA,sourceNode->point) <= distB;
	} else {
		distB = CGAL::abs( intersection.getIntersection().y() - data.getEdge(edgeIdxA).point(0).y() );
		distB=distB*distB;
	}


	auto newNodeIdx = wf.addNode(intersection.getIntersection(),distB);
	auto newNode    = &wf.nodes[newNodeIdx];

	LOG(INFO) << " NEW IDX: " << newNodeIdx << "/" << wf.nodes.size();

	/* distinguish in which direction the ray points and add the arc accordingly */
	uint newArcIdx = 0;
	if(bis.isLine() || fromAtoB ) {
		newArcIdx  = wf.addArc(sourceNodeIdx,newNodeIdx,edgeIdxA,edgeIdxB,bis.is_vertical(),bis.is_horizontal());
	} else {
		newArcIdx  = wf.addArc(newNodeIdx,sourceNodeIdx,edgeIdxB,edgeIdxA,bis.is_vertical(),bis.is_horizontal());
	}

	/* update the targets of the relevant arcs */
	for(auto arcIdx : intersection.getArcs()) {
		LOG(INFO) << "update the targets of " << arcIdx; fflush(stdout);
		if(arcIdx < MAX) {
			auto arc = wf.getArc(arcIdx);
			/* TODO: parallel-bisectors, direction unclear */
			LOG(INFO) << "update arc target " << arcIdx << " to end at " << newNodeIdx;
			updateArcTarget(arcIdx,edgeIdxA,newNodeIdx,intersection.getIntersection());
		}
	}

	auto arc = wf.getArc(intersection.getFirstArcIdx());
	if(bis.is_vertical() && arc->is_vertical()) {
		addGhostNode = true;
	}

	return newNodeIdx;
}
uint Skeleton::getAVerticalArc(const Intersection& intersection) const {
	for(auto arcIdx : intersection.getArcs()) {
		if(wf.getArc(arcIdx)->isAA()) {
			return arcIdx;
		}
	}
	return MAX;
}

bool Skeleton::intersectionHasVerticalArc(const IntersectionPair& pair) const {
	uint upperArcIdx = getAVerticalArc(pair.first);
	uint lowerArcIdx = getAVerticalArc(pair.second);

	return upperArcIdx != MAX || lowerArcIdx != MAX;
}

bool Skeleton::intersectionHasVerticalCoallignedArc(const IntersectionPair& pair) const {
	uint upperArcIdx = getAVerticalArc(pair.first);
	uint lowerArcIdx = getAVerticalArc(pair.second);

	if(upperArcIdx != MAX && lowerArcIdx != MAX) {
		auto lowerArc = wf.getArc(lowerArcIdx);
		auto upperArc = wf.getArc(upperArcIdx);
		return lowerArc->isCollinear(*upperArc);
	}
	return false;
}

bool Skeleton::hasPathReachedPoint(const MonotonePathTraversal& path, const Point& P) const {
	auto arc = wf.getArc(path);
	uint leftNodeArcIdx = wf.getLeftmostNodeIdxOfArc(*arc);

	LOG(INFO) << wf.upperPath;
	LOG(INFO) << wf.lowerPath;

	Point pointArc = wf.nodes[leftNodeArcIdx].point;

	Point pArcProj = data.monotonicityLine.projection(pointArc);
	Point pProj    = data.monotonicityLine.projection(P);
	return data.monotoneSmaller(P,pointArc) &&
		   ( !arc->isRay() || ( arc->isRay() && !data.rayPointsLeft(arc->ray) ) );
}

/* we may have walked one arc to far on one path */
void Skeleton::CheckAndResetPath(MonotonePathTraversal* path, const MonotonePathTraversal& pathBackup, const Point& p) {
	if(path->currentArcIdx == MAX) {return;}

	if(*path != pathBackup && hasPathReachedPoint(*path,p)) {
		auto arcA = wf.getArc(pathBackup.currentArcIdx);
		auto arcB = wf.getArc(pathBackup.oppositeArcIdx);

		LOG(INFO) << "check backup path: " << pathBackup;
		LOG(INFO) << "current: " << *arcA << ", opposite: " << *arcB;
		LOG(INFO) << std::boolalpha << "disabled: current " << arcA->isDisable() << " opposite: " << arcB->isDisable();

		if(!arcA->isDisable() && !arcB->isDisable()) {
			/* reset the other path with pathBackup */
			path->set(pathBackup);
			if(path->upperChain) {
				upperChainIndex = path->edgeIdx;
			} else {
				lowerChainIndex = path->edgeIdx;
			}
			LOG(WARNING) << "After Backup: " << *path;
		} else {
			LOG(INFO) << "CheckAndResetPath: disabled arc involved (not restoring).";
		}
	}
}

bool Skeleton::hasEquidistantInputEdges(const Arc& arc, const Bisector& bis) const {
	std::set<uint> edgeIndicesSet = {arc.leftEdgeIdx,arc.rightEdgeIdx};
	std::vector<uint> bisIndices = {bis.eIdxA,bis.eIdxB};

	if(arc.isAA() || bis.isAA()) {

		for(auto nidx : {arc.firstNodeIdx,arc.secondNodeIdx}) {
			if(nidx != MAX) {
				auto node = wf.getNode(nidx);
				for(auto aidx : node->arcs) {
					auto arcIt = wf.getArc(aidx);
					edgeIndicesSet.insert(arcIt->leftEdgeIdx);
					edgeIndicesSet.insert(arcIt->rightEdgeIdx);
				}
			}
		}

		std::vector<Edge> edges;
		std::vector<Edge> bisEdges;
		for(auto idx : edgeIndicesSet) {
			LOG(INFO) << "edx: " << idx;
			edges.push_back(data.getEdge(idx));
		}
		for(auto idx : bisIndices) {
			LOG(INFO) << "bisedx: " << idx;
			bisEdges.push_back(data.getEdge(idx));
		}
		if(edges.size() < 2) {
			return false;
		}
		std::vector<Exact> distances;
		for(auto iA : bisEdges) {
			for(auto iB : edges) {
				if(!data.isEdgeCollinear(iA,iB) && isLinesParallel(iA,iB) ) {
					auto dist = CGAL::squared_distance(iA.supporting_line(),iB.supporting_line());
					if(dist > Exact(0)) {
						LOG(INFO) << "d: " << dist;
						distances.push_back(dist);
					}
				}
			}
		}

		if(distances.size() > 1) {
			LOG(INFO) << "number of computed distances: " << distances.size();
			return distances[0] == distances[1];
		}
	}
	return false;
}

bool Skeleton::areNextInputEdgesCollinear() const {
	if(EndOfUpperChain() || EndOfLowerChain()) {
		return false;
	}

	auto arc_u = wf.getArc(wf.upperPath);
	auto arc_l = wf.getArc(wf.lowerPath);
	auto nextUpperEdgeIdx = (arc_u->leftEdgeIdx != upperChainIndex) ? arc_u->leftEdgeIdx : arc_u->rightEdgeIdx;
	auto nextLowerEdgeIdx = (arc_l->leftEdgeIdx != lowerChainIndex) ? arc_l->leftEdgeIdx : arc_l->rightEdgeIdx;

	LOG(INFO) << "next edge inidices: " <<nextUpperEdgeIdx  << " - " << nextLowerEdgeIdx;

	auto upperEdge = data.getEdge(nextUpperEdgeIdx);
	auto lowerEdge = data.getEdge(nextLowerEdgeIdx);
	Point A = upperEdge.point(0);
	Point B = upperEdge.point(1);
	Point C = (lowerEdge.point(1) + 2*lowerEdge.to_vector());

	if(CGAL::collinear(A,B,C)) {
		LOG(INFO) << "edges are collinear: " << upperEdge << " and " << lowerEdge;
		LOG(INFO) << "Points: " << A << " " <<  B << " " << C;
		return true;
	}

	return false;
}

void Skeleton::handleSourceGhostNode(Bisector& bis, IntersectionPair& pair) {
	if(sourceNode->isGhostNode()) {
		LOG(INFO) << "handleSourceGhostNode";
		/* find the intersection of the next arcs (upper and lower) */
		assert(pair.first.size() < 2);
		assert(pair.second.size() < 2);

		auto upperArc = wf.getArc(*pair.first.getArcs().begin());
		auto lowerArc = wf.getArc(*pair.second.getArcs().begin());

		/* we have a ghost node to handle, i.e., next intersection gives us the 'height' '
		 * since the actual height is not known yet  */

		Point newNodePoint = intersectArcArc(*upperArc,*lowerArc);

		if(newNodePoint == INFPOINT) {
			newNodePoint = pair.first.getIntersection();
		}
		if(newNodePoint == INFPOINT) {
			newNodePoint = pair.second.getIntersection();
		}

		if(newNodePoint != INFPOINT) {
			pair.first.setIntersection(newNodePoint);
			pair.second.setIntersection(newNodePoint);

			uint chosenArcIdx = MAX;
			auto PGhost = Point(sourceNode->point.x(),newNodePoint.y());
			std::vector<uint> verticalArcs;
			for(auto aIdx : sourceNode->arcs) {
				auto arc = wf.getArc(aIdx);
				if(arc->is_vertical()) {
					Exact yMin = arc->point(0).y();
					Exact yMax = arc->point(1).y();
					if(yMin > yMax) {std::swap(yMin,yMax);}
					Exact yP = PGhost.y();
					if(yP > yMin && yP < yMax) {
						verticalArcs.push_back(aIdx);
					}
				}
			}

			LOG(INFO) << "verticalArcs.size: " << verticalArcs.size();

			if(verticalArcs.size() < 1) {
				return;
			}

			chosenArcIdx = *verticalArcs.begin();
			auto arc = wf.getArc(chosenArcIdx);
			PGhost = Point(arc->point(0).x(),newNodePoint.y());

			if(sourceNode->degree() > 2) {
				auto newSourceNodeIdx = wf.addNode(PGhost,sourceNode->time);

				if(wf.isEdgeOnLowerChain(arc->leftEdgeIdx)) {
					wf.addArc(newSourceNodeIdx,sourceNodeIdx,arc->leftEdgeIdx,upperChainIndex,arc->is_horizontal(),arc->is_vertical());
				} else /* edgeIdx on upper chain */ {
					wf.addArc(newSourceNodeIdx,sourceNodeIdx,lowerChainIndex,arc->rightEdgeIdx,arc->is_horizontal(),arc->is_vertical());
				}

				sourceNode->removeArc(chosenArcIdx);
				arc->secondNodeIdx = newSourceNodeIdx;
				wf.updateArcNewNode(chosenArcIdx,newSourceNodeIdx);

				sourceNodeIdx = newSourceNodeIdx;
				sourceNode = wf.getNode(sourceNodeIdx);
				sourceNode->arcs.push_back(chosenArcIdx);
			} else {
				sourceNode->point = PGhost;
				for(auto arcIdx : sourceNode->arcs) {
					wf.updateArcNewNode(arcIdx,sourceNodeIdx);
				}
			}
		}
	}
}

bool Skeleton::handleGhostVertex(const MonotonePathTraversal& path,  Bisector& bis, Intersection& intersection) {
	auto arc = wf.getArc(path.currentArcIdx);
	if(bis.isParallel() && (wf.isArcPerpendicular(*arc))) {
		/* if we have collinear bisectors/arcs then three input edges must be equidistant */
		if(hasEquidistantInputEdges(*arc,bis)) {
			LOG(WARNING) << "possible ghost arc ahead!";
			/* we add a ghost node that lies between the last added node and the closest endpoint of arc */
			/* we move this point later when we know where it lies */
			Point pA = arc->point(0);
			Point pB = arc->point(1);
			Point pS = sourceNode->point;
			LOG(INFO) << "point A " << pA << " B " << pB << " S " << pS;
			Point closestPoint = INFPOINT;
			if(data.monotonicityLine.to_vector().x() > Exact(0)) {
				closestPoint = (CGAL::abs(pS.y()-pA.y()) <= CGAL::abs(pS.y()-pB.y())) ? pA : pB;
			} else {
				closestPoint = (CGAL::abs(pS.x()-pA.x()) <= CGAL::abs(pS.x()-pB.x())) ? pA : pB;
			}
			LOG(INFO) << "<- set addGhostNode true (handleGhostVertex)";
			addGhostNode = true;
			Point midPoint = CGAL::midpoint(pS,closestPoint);
			intersection.add(midPoint,path.currentArcIdx);
			intersection.setDone();
			return true;
		}
	}

	return false;
}

void Skeleton::updateArcTarget(const uint& arcIdx, const uint& edgeIdx, const int& secondNodeIdx, const Point& edgeEndPoint) {
	auto arc = &wf.arcList[arcIdx];

	if(arc->isDisable()) {return;}

	/* remove or disable rest of path that we broke? */
	/* we act in case the degree of the second node is <3, otherwise for >2 the path is still accessible from another route */
	if(arc->type == ArcType::NORMAL) {
		removePath(arcIdx, edgeIdx);
	}


	auto newNode = &wf.nodes[secondNodeIdx];

	if(arc->point(0) == newNode->point) {
		arc->disable();
		return;
	} else if(arc->isRay()) {
		arc->type = ArcType::NORMAL;
		arc->edge = Edge(arc->ray.source(),edgeEndPoint);
		arc->ray = Ray();
	} else if(arc->isEdge()){
		arc->edge = Edge(arc->edge.source(),edgeEndPoint);
	}

	arc->secondNodeIdx = secondNodeIdx;

	newNode->arcs.push_back(arcIdx);
}

bool Skeleton::hasCollinearEdges(const Arc& arcA, const Arc& arcB) const {
	auto idxA = (upperChainIndex == arcA.leftEdgeIdx || lowerChainIndex == arcA.leftEdgeIdx) ? arcA.rightEdgeIdx : arcA.leftEdgeIdx;
	auto idxB = (upperChainIndex == arcB.leftEdgeIdx || lowerChainIndex == arcB.leftEdgeIdx) ? arcB.rightEdgeIdx : arcB.leftEdgeIdx;
	return data.isEdgeCollinear(idxA,idxB);
}

bool Skeleton::removePath(const uint& arcIdx, const uint& edgeIdx)  {
	auto arcIdxIt = arcIdx;

	while(true) {
		auto arc  = &wf.arcList[arcIdxIt];

		if(arc->type == ArcType::RAY || arc->secondNodeIdx == MAX) {
			if(arcIdx != arcIdxIt) {
				arc->disable();
			}
			return false;
		}

		if(arcIdx != arcIdxIt) {
			arc->disable();
		}

		LOG(INFO) << " arc-start: " << arc->firstNodeIdx <<  " arc-endpoint: " << arc->secondNodeIdx << " ";

		auto secondNode = &wf.nodes[arc->secondNodeIdx];
		auto arcs = &secondNode->arcs;

		LOG(INFO) << " arcs:" << arcs->size() << " ";

		/* remove reference to 'arcIdx' from node */
		if(!secondNode->removeArc(arcIdxIt)) {return false;}
		if(secondNode->isGhostNode())        {return false;}

		if(arcs->size() < 2) {
			/* degree one means that only one path is left */
			bool nextArcFound = false;
			for(auto a : *arcs) {
				if(a != arcIdxIt) {
					auto nextArc = &wf.arcList[a];

					if(	 arc->secondNodeIdx == nextArc->firstNodeIdx
					  && (nextArc->leftEdgeIdx == arc->leftEdgeIdx || nextArc->rightEdgeIdx == arc->rightEdgeIdx )
					) {
						/* iterate arcs */
						arcIdxIt = a;
						nextArcFound = true;
						LOG(INFO) << " - " << arcIdxIt << " is adjacent! ";
					}
				}
			}
			if(!nextArcFound) {
				return false;
			}
		}
	}
	LOG(WARNING) << " no more arc on path! ";
	return false;

//		/* TODO: fix this: */
//		auto it = std::lower_bound(arcs->begin(),arcs->end(),arcIdx,ArcCmp(wf.arcList));
//		assert(it != arcs->end());
//
//		auto nextArc = &wf.arcList[*(++it)];
//		if(nextArc->leftEdgeIdx == edgeIdx || nextArc->rightEdgeIdx == edgeIdx) {
//			nextArcIdx = *it;
//			return true;
//		} else {
//			--it; --it;
//			nextArc = &wf.arcList[*(it)];
//			if(nextArc->leftEdgeIdx == edgeIdx || nextArc->rightEdgeIdx == edgeIdx) {
//				nextArcIdx = *it;
//				return true;
//			} else {
//				LOG(WARNING) << " OH NO! SOME SORTING FAILED!! ";
//				return false;
//			}
//		}

}


void Skeleton::printSkeleton() const {
	std::cout << "# Lower Skeleton, Edge 'soup' " << std::endl;

	for(auto n : wf.nodes) {
		std::cout << "v " << n.point << " 0.0"<< std::endl;
	}

	for(auto a : wf.arcList) {
		std::cout << "l " << a.firstNodeIdx+1 << " " << a.secondNodeIdx+1 << std::endl;
	}
}

void Skeleton::writeOBJ(const Config& cfg) const {
	if(cfg.outputType == OutputType::OBJ) {
		double xt = 0.0, yt = 0.0, zt = 0.0, xm = 1.0, ym = 1.0, zm = 1.0;
		if(cfg.normalize) {
			getNormalizer(data.bbox,xt,xm,yt,ym,zt,zm);
			zm = 0.1;
		}
		uint errorCnt = 20;

		std::ofstream outfile (cfg.outputFileName,std::ofstream::binary);
		outfile << "# OBJ-File autogenerated by monos from file ("
				<< cfg.fileName << ") - "
				<< currentTimeStamp() <<  std::endl;

		/* write points/nodes into file */
		for(auto n : wf.nodes) {
			double x = (n.point.x().doubleValue() - xt) * xm;
			double y = (n.point.y().doubleValue() - yt) * ym;
			outfile << "v " << x << " " << y << " " << CGAL::sqrt(n.time).doubleValue() * zm << std::endl;
		}

		/* write faces induced by the skeleton into file */
		for(uint edgeIdx = 0; edgeIdx < data.getPolygon().size(); ++edgeIdx) {
			auto e = data.e(edgeIdx);
			std::vector<Node*> tN = {{&wf.nodes[e[0]],&wf.nodes[e[1]]}};

			/* we walk from the right (index 1) terminal node along the boudnary of the
			 * induced face to the first (index 0) terminal node */
			auto arcIdx = tN[1]->arcs.front();
			auto srcNodeIdx = e[1];
			auto arcIt = wf.getArc(arcIdx);

			outfile << "f " << e[0]+1 << " " << e[1]+1;

			do {
				auto nextNodeIdx = arcIt->getSecondNodeIdx(srcNodeIdx);

				if(nextNodeIdx == INFINITY) {LOG(WARNING) << "infinite node in list!"; break;}

				/* +1 is the standard OBJ offset for references */
				outfile << " " << nextNodeIdx+1;

				auto n = &wf.nodes[nextNodeIdx];
				bool found = false;
				for(auto newArcIdx : n->arcs) {
					if(arcIdx != newArcIdx) {
						arcIt = wf.getArc(newArcIdx);
						if(arcIt->isDisable()) {continue;}
						if(arcIt->leftEdgeIdx == edgeIdx || arcIt->rightEdgeIdx == edgeIdx) {
							found  = true;
							arcIdx = newArcIdx;
							break;
						}
					}
				}

				if(!n->isTerminal() && !found) {
					LOG(WARNING) << "did not find a next arc! current node: " << nextNodeIdx << " " << *n;
					--errorCnt;
				}

				srcNodeIdx = nextNodeIdx;

			} while(arcIt->secondNodeIdx != e[0] && arcIt->firstNodeIdx != e[0] && errorCnt > 0);

			outfile << std::endl;

		}

		outfile.close();
	}
}
