
#include "Skeleton.h"


void Skeleton::initMerge() {
	upperChainIndex    	= upperChain.back();
	lowerChainIndex    	= lowerChain.front();
	sourceNodeIdx 	   	= wf.pathFinder[lowerChainIndex].a;
	sourceNode    		= &wf.nodes[sourceNodeIdx];
	newNodeIdx    		= sourceNodeIdx;

}

/* add last are, connect to end node! */
void Skeleton::finishMerge() {
	ul lastTNodeIdx = data.e(lowerChain.back()).v;
	Segment e(wf.getNode(lastTNodeIdx)->point, wf.getNode(sourceNodeIdx)->point);
	wf.addArc(lastTNodeIdx,sourceNodeIdx,lowerChainIndex,upperChainIndex);
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

	auto bisLine = data.simpleBisector(upperChainIndex,lowerChainIndex);

	/* correct direction if necessary */
	const auto& Pa = bisLine.point(0);	const auto Pb = Pa + bisLine.to_vector();
	if(!data.monotoneSmaller(Pa,Pb)) {bisLine = bisLine.opposite();}
	const auto bis = Ray(sourceNode->point,bisLine.direction());

	LOG(INFO) << "Bisector-dir: " << bis.direction();

	/* setup intersection call */
	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	IntersectionPair intersectionPair = findNextIntersectingArc(bis);

	LOG(INFO) << "intersection upper: " << intersectionPair.first << std::endl
			  << "intersection lower: " << intersectionPair.second;

	newNodeIdx = handleMerge(intersectionPair, bis);


	/* we iterate the sourcenode to the newly added node and go on merging */
	LOG(INFO) << "changing idx from old: " << sourceNodeIdx << " to " << newNodeIdx;
	sourceNodeIdx = newNodeIdx;
	sourceNode = &wf.nodes[sourceNodeIdx];

	LOG(INFO) << "";
	LOG(INFO) << "############################################## END STEP ###########################";
	LOG(INFO) << "";

	return !EndOfBothChains();
}

/* finds the next arc(s) intersected by the bisector 'bis' that lie closest to 'sourceNode'
 * in respect to the 'monotonicityLine' */
IntersectionPair Skeleton::findNextIntersectingArc(const Ray& bis) {
	assert(sourceNode != nullptr);
	Point Pu = INFPOINT;
	Point Pl = INFPOINT;

	/* holds the arcs we find in this search */
	if(initPathForEdge(ChainType::UPPER,upperChainIndex)) {
		Arc* upperArc = nullptr;
		do {
			upperArc = wf.getArc(wf.upperPath.currentArcIdx);
			LOG(INFO) << "upper checking " << *upperArc;
			wf.upperPath.currentArcIdx = wf.getNextArcIdx(wf.upperPath,*upperArc);
			if(isIntersecting(bis,*upperArc)) {
				wf.upperPath.currentArcIdx = upperArc->id;
				break;
			}
		} while(wf.upperPath.currentArcIdx != MAX);

		if(wf.upperPath.currentArcIdx != MAX) {
			if(upperArc->isEdge()) {
				Pu = intersectElements(bis,upperArc->segment);
			} else {
				Pu = intersectElements(bis,upperArc->ray);
			}
			LOG(INFO) << "upperPath found intersection with " << upperArc->id << ", " << wf.upperPath.currentArcIdx << " in pathvar";
		}
	}

	if(initPathForEdge(ChainType::LOWER,lowerChainIndex)) {
		Arc* lowerArc = nullptr;
		do {
			lowerArc = wf.getArc(wf.lowerPath.currentArcIdx);
			LOG(INFO) << "lower checking " << *lowerArc;
			wf.lowerPath.currentArcIdx = wf.getNextArcIdx(wf.lowerPath,*lowerArc);
			if(isIntersecting(bis,*lowerArc)) {
				wf.lowerPath.currentArcIdx = lowerArc->id;
				break;
			}
		} while(wf.lowerPath.currentArcIdx != MAX);

		if(wf.lowerPath.currentArcIdx != MAX) {
			if(lowerArc->isEdge()) {
				Pl = intersectElements(bis,lowerArc->segment);
			} else {
				Pl = intersectElements(bis,lowerArc->ray);
			}
			LOG(INFO) << "lowerPath found intersection with " << lowerArc->id << ", " << wf.lowerPath.currentArcIdx << " in pathvar";
		}
	}

	return std::make_pair(Pu,Pl);
}

bool Skeleton::initPathForEdge(ChainType type, const ul& edgeIdx) {
	/* set the upperPath/lowerPath in 'wf' */
	LOG(INFO) << "initPathForEdge " << edgeIdx;

	if(EndOfChain(type)) {return false;}

	const Node& terminalNode   = (ChainType::UPPER == type) ? wf.getTerminalNodeForVertex(data.e(edgeIdx).u) : wf.getTerminalNodeForVertex(data.e(edgeIdx).v);
	auto& path = (ChainType::UPPER == type) ? wf.upperPath : wf.lowerPath;

	path.edgeIdx = edgeIdx;
	path.currentArcIdx = terminalNode.arcs.front();

	const Node* finalNode = (ChainType::UPPER == type) ? wf.getNode(wf.pathFinder[edgeIdx].a) : wf.getNode(wf.pathFinder[edgeIdx].a);
	path.finalArcIdx = MAX;
	for(auto a : finalNode->arcs) {
		const auto& arc = getArc(a);
		if(ChainType::UPPER == type && arc.rightEdgeIdx == edgeIdx) {
			path.finalArcIdx = a;
			if(arc.isRay()) {
				break;
			}
		}
	}
	LOG(INFO) << "path init done " << path.currentArcIdx; fflush(stdout);
	return true;
}


ul Skeleton::handleMerge(const IntersectionPair& intersectionPair, const Ray& bis) {
	const Point& Pu = intersectionPair.first;
	const Point& Pl = intersectionPair.second;

	Point P = INFPOINT;
	ChainType winner;

	if(Pu != INFPOINT && Pl != INFPOINT) {
		winner = data.monotoneSmaller(bis.supporting_line(),Pu,Pl) ? ChainType::UPPER : ChainType::LOWER;
		P = (winner == ChainType::UPPER) ? Pu : Pl;
	} else if(Pu != INFPOINT) {
		winner = ChainType::UPPER;
		P = Pu;
	} else {
		assert(Pl != INFPOINT);
		winner = ChainType::LOWER;
		P = Pl;
	}

	LOG(INFO) << "handleMerge Upper current arc idx " << wf.upperPath.currentArcIdx;
	LOG(INFO) << "handleMerge Lower current arc idx " << wf.lowerPath.currentArcIdx;

	const auto& edgeIdx = (winner == ChainType::UPPER) ? upperChainIndex : lowerChainIndex;
	const auto& path    = (winner == ChainType::UPPER) ? wf.upperPath    : wf.lowerPath;

	assert(P != INFPOINT);

	NT dist = data.normalDistance(edgeIdx,P);


	const auto newNodeIdx 	= wf.addNode(P,dist);
	const ul newArcIdx 		= wf.addArc(sourceNodeIdx,newNodeIdx,upperChainIndex,lowerChainIndex);

	auto& intersArc = getArc(path.currentArcIdx);

	/* update the targets of the relevant arcs */
	LOG(INFO) << "before update of " << path.currentArcIdx;
	updateArcTarget(path.currentArcIdx,edgeIdx,newNodeIdx,P);

	if(winner == ChainType::UPPER) {
		upperChainIndex = intersArc.leftEdgeIdx;
		LOG(INFO) << "handleMerge: set upper chain to " << upperChainIndex;
	} else {
		lowerChainIndex = intersArc.rightEdgeIdx;
		LOG(INFO) << "handleMerge: set lower chain to " << lowerChainIndex;
	}

	return newNodeIdx;
}


void Skeleton::updateArcTarget(const ul& arcIdx, const ul& edgeIdx, const int& secondNodeIdx, const Point& edgeEndPoint) {
	auto arc = &wf.arcList[arcIdx];

	if(arc->isDisable()) {return;}

	/* remove or disable rest of path that we broke? */
	/* we act in case the degree of the second node is <3, otherwise for >2 the path is still accessible from another route */
	if(arc->type == ArcType::NORMAL) {
		removePath(arcIdx, edgeIdx);
	}

	LOG(INFO) << "in update " << *arc;

	auto newNode = &wf.nodes[secondNodeIdx];

	if(arc->point(0) == newNode->point) {
		arc->disable();
		return;
	} else if(arc->isRay()) {
		arc->type = ArcType::NORMAL;
		arc->segment = Segment(arc->ray.source(),edgeEndPoint);
		arc->ray = Ray();
	} else if(arc->isEdge()){
		arc->segment = Segment(arc->segment.source(),edgeEndPoint);
	}

	arc->secondNodeIdx = secondNodeIdx;

	newNode->arcs.push_back(arcIdx);

	LOG(INFO) << "after update" << *arc;
}

bool Skeleton::removePath(const ul& arcIdx, const ul& edgeIdx)  {
	auto arcIdxIt = arcIdx;
	LOG(INFO) << "in removePath";

	while(true) {
		auto arc  = &wf.arcList[arcIdxIt];

		if(arcIdx != arcIdxIt) {
			arc->disable();
		}

		LOG(INFO) << *arc;
		LOG(INFO) << " arc-start: " << arc->firstNodeIdx <<  " arc-endpoint: " << arc->secondNodeIdx << " ";

		if(arc->type == ArcType::RAY || arc->secondNodeIdx == MAX) {
			return false;
		}

		auto secondNode = &wf.nodes[arc->secondNodeIdx];
		auto arcs = &secondNode->arcs;

		/* remove reference to 'arcIdx' from node */
		if(!secondNode->removeArc(arcIdxIt)) {return false;}
		if(secondNode->isGhostNode())        {return false;}


		if(arcs->size() < 2 && !secondNode->isTerminal()) {
			LOG(INFO) << " arcs:" << arcs->size() << " " << *secondNode;
			/* degree one means that only one path is left */
			bool nextArcFound = false;
			for(auto a : *arcs) {
				if(a != arcIdxIt) {
					/* iterate arcs */
					arcIdxIt = a;
					nextArcFound = true;
					LOG(INFO) << " - " << arcIdxIt << " is adjacent! ";
				}
			}
			if(!nextArcFound) {
				return false;
			}
		}
	}
	LOG(WARNING) << " no more arc on path! ";
	return false;


}


void Skeleton::writeOBJ(const Config& cfg) const {
	double xt = 0.0, yt = 0.0, zt = 0.0, xm = 1.0, ym = 1.0, zm = 1.0;
	if(cfg.normalize) {
		getNormalizer(*data.bbox,xt,xm,yt,ym,zt,zm);
		zm = 0.1;
	} else {
		xm = 1.0/OBJSCALE;
		ym = 1.0/OBJSCALE;
	}

	LOG(INFO) << "writing OBJ: " << wf.nodes.size(); fflush(stdout);

	zm /= OBJSCALE;
	ul errorCnt = 20;

	std::ofstream outfile (cfg.outputFileName,std::ofstream::binary);
	outfile << "# OBJ-File autogenerated by monos from file ("
			<< cfg.fileName << ") - "
			<< currentTimeStamp() <<  std::endl;

	/* write points/nodes into file */
	for(auto n : wf.nodes) {
		double x = (n.point.x().doubleValue() - xt)   * xm;
		double y = (n.point.y().doubleValue() - yt)   * ym;
		double z = CGAL::sqrt(n.time).doubleValue()   * zm;
		outfile << "v " << x << " " << y << " " << z << std::endl;
	}

	/* write faces induced by the skeleton into file */
	for(ul edgeIdx = 0; edgeIdx < data.getPolygon().size(); ++edgeIdx) {
		auto e = data.e(edgeIdx);
		std::vector<Node*> tN = {{&wf.nodes[e.u],&wf.nodes[e.v]}};

		/* we walk from the right (index 1) terminal node along the boudnary of the
		 * induced face to the first (index 0) terminal node */
		auto arcIdx = tN[1]->arcs.front();
		auto srcNodeIdx = e.u;
		auto arcIt = wf.getArc(arcIdx);

		outfile << "f " << e.u+1; // << " " << e.v+1;

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

		} while(arcIt->secondNodeIdx != e.u && arcIt->firstNodeIdx != e.u && errorCnt > 0);

		outfile << std::endl;
	}

	outfile.close();
}
