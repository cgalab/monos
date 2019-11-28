
#include "Skeleton.h"


void Skeleton::initMerge() {
	upperChainIndex    	= upperChain.back();
	lowerChainIndex    	= lowerChain.front();
	sourceNodeIdx 	   	= wf.pathFinder[lowerChainIndex].a;
	sourceNode    		= &wf.nodes[sourceNodeIdx];
	newNodeIdx    		= sourceNodeIdx;

	initPathForEdge(ChainType::UPPER);
	initPathForEdge(ChainType::LOWER);
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
//	const auto& Pa = bisLine.point(0);	const auto Pb = Pa + bisLine.to_vector();
	if(ORIGIN > ORIGIN + bisLine.to_vector()) {bisLine = bisLine.opposite();}

	LOG(INFO) << "Bisector-dir: " << bisLine.direction();

	/* setup intersection call */
	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	IntersectionPair intersectionPair = findNextIntersectingArc(bisLine);

	newNodeIdx = handleMerge(intersectionPair);


	/* we iterate the sourcenode to the newly added node and go on merging */
	LOG(INFO) << "changing idx from old: " << sourceNodeIdx << " to " << newNodeIdx;
	sourceNodeIdx = newNodeIdx;
	sourceNode = &wf.nodes[sourceNodeIdx];

	LOG(INFO) << "";
	LOG(INFO) << "############################################## END STEP ###########################";
	LOG(INFO) << "  ( next is ui: " << upperChainIndex << ", a: " << upperPath << " -- li: " << lowerChainIndex <<", a: " << lowerPath << " )   ";

	return !EndOfBothChains();
}

/* finds the next arc(s) intersected by the bisector 'bis' that lie closest to 'sourceNode'
 * in respect to the 'monotonicityLine' */
IntersectionPair Skeleton::findNextIntersectingArc(const Line& bis) {
	assert(sourceNode != nullptr);
	Point Pu = INFPOINT; Point Pl = INFPOINT;
	Point uPa, uPb, lPa, lPb;
	bool doneU = false, doneL = false;
	bool iterateForwardU = true, iterateForwardL = true;
	Arc* upperArc = nullptr; Arc* lowerArc = nullptr;
	ChainType searchChain;

	if(EndOfUpperChain()) {doneU = true; LOG(WARNING) << "end of upper chain!";}
	if(EndOfLowerChain()) {doneL = true; LOG(WARNING) << "end of lower chain!";}

	if(!doneU) {
		iterateForwardU = decideDirection(ChainType::UPPER,bis);
		upperArc = wf.getArc(upperPath);
	}

	if(!doneL) {
		iterateForwardL = decideDirection(ChainType::LOWER,bis);
		lowerArc = wf.getArc(lowerPath);
	}

	do {
		if(!doneU && !doneL) {
			uPa = upperArc->source(); uPb = upperArc->target();
			lPa = lowerArc->source(); lPb = lowerArc->target();
			if(uPb < uPa) {std::swap(uPa, uPb);}
			if(lPb < lPa) {std::swap(lPa, lPb);}

			searchChain = (uPa < lPa) ? ChainType::UPPER : ChainType::LOWER;
		}

		if(doneU) {searchChain = ChainType::LOWER;}
		if(doneL) {searchChain = ChainType::UPPER;}

		if(!doneU && searchChain == ChainType::UPPER && !EndOfUpperChain()) {
			LOG(INFO) << "upper checking " << *upperArc;
			if(isIntersecting(bis,*upperArc)) {
				Pu = intersectElements(bis,upperArc->supporting_line());
				doneU = true;
			} else {
				upperPath = wf.getNextArcIdx(upperPath,iterateForwardU,upperChainIndex);
				if(upperPath == MAX) {
					doneU = true;
					upperPath = upperArc->id;
				} else {
					upperArc = wf.getArc(upperPath);
				}
			}
		}

		if(!doneL && searchChain == ChainType::LOWER && !EndOfLowerChain()) {
			LOG(INFO) << "lower checking " << *lowerArc;

			if(isIntersecting(bis,*lowerArc)) {
				Pl = intersectElements(bis,lowerArc->supporting_line());
				doneL = true;
			} else {
				lowerPath = wf.getNextArcIdx(lowerPath,iterateForwardL,lowerChainIndex);
				if(lowerPath == MAX) {
					doneL = true;
					lowerPath = lowerArc->id;
				} else {
					lowerArc = wf.getArc(lowerPath);
				}
			}
		}
	} while(!doneU || !doneL);

	return std::make_pair(Pu,Pl);
}

void Skeleton::initPathForEdge(ChainType type) {
	/* set the upperPath/lowerPath in 'wf' */
	const ul& edgeIdx = (ChainType::UPPER == type) ? upperChainIndex : lowerChainIndex;
	const Node& terminalNode = (ChainType::UPPER == type) ? wf.getTerminalNodeForVertex(data.e(edgeIdx).u) : wf.getTerminalNodeForVertex(data.e(edgeIdx).v);
	auto& path        = (ChainType::UPPER == type) ? upperPath : lowerPath;

	path = (!EndOfChain(type)) ? terminalNode.arcs.front() : MAX;

	LOG(INFO) << "initPathForEdge " << edgeIdx << " with arc idx: " << path;
}


ul Skeleton::handleMerge(const IntersectionPair& intersectionPair) {
	const Point& Pu = intersectionPair.first;
	const Point& Pl = intersectionPair.second;

	ul edgeIdx  = upperChainIndex;
	ul path     = upperPath;
	Point P     = Pu;

	ChainType winner;

	if(Pu != INFPOINT && Pl != INFPOINT) {
		winner = (Pu < Pl) ? ChainType::UPPER : ChainType::LOWER;
	} else if(Pu != INFPOINT) {
		winner = ChainType::UPPER;
	} else {
		assert(Pl != INFPOINT);
		winner = ChainType::LOWER;
	}

	if(winner == ChainType::LOWER) {
		path = lowerPath;
		edgeIdx = lowerChainIndex;
		P = Pl;
	}

	assert(P != INFPOINT);

	NT dist = data.normalDistance(edgeIdx,P);

	const ul newNodeIdx = wf.addNode(P,dist);
	const ul newArcIdx 	= wf.addArc(sourceNodeIdx,newNodeIdx,upperChainIndex,lowerChainIndex);

	const auto* intersArc = wf.getArc(path);

	/* update the targets of the relevant arcs */
	LOG(INFO) << "before update of " << path;
	updateArcTarget(path,edgeIdx,newNodeIdx,P);

	if(edgeIdx == upperChainIndex) {
		upperChainIndex = intersArc->leftEdgeIdx;
		LOG(INFO) << "handleMerge: set upper chain to " << upperChainIndex << " arc: " << upperPath;
	} else {
		lowerChainIndex = intersArc->rightEdgeIdx;
		LOG(INFO) << "handleMerge: set lower chain to " << lowerChainIndex << " arc: " << lowerPath;
	}

	initPathForEdge(winner);

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
	} else {
		wf.arcList[arcIdx] = Arc(ArcType::NORMAL, arc->firstNodeIdx, secondNodeIdx,
								arc->leftEdgeIdx, arc->rightEdgeIdx, arc->id,
								Segment(arc->source(),edgeEndPoint)
							 );
	}

	newNode->arcs.push_back(arcIdx);

	LOG(INFO) << "after update" << *arc;
}

bool Skeleton::decideDirection(ChainType type, const Line& bis) const {
	Arc* arc = nullptr;
	if(type == ChainType::UPPER) {
		arc = wf.getArc(upperPath);
		if(arc->isRay()) {return false;}
	} else /* ChainType::LOWER */ {
		arc = wf.getArc(lowerPath);
		if(arc->isRay()) {return false;}
	}

	if(type == ChainType::UPPER) {
		return bis.has_on_positive_side(arc->source());
	} else {
		return bis.has_on_negative_side(arc->source());
	}
}

void Skeleton::removePath(const ul& arcIdx, const ul& edgeIdx)  {
	auto arcIdxIt = arcIdx;

	while(true) {
		auto arc  = &wf.arcList[arcIdxIt];

		if(arcIdx != arcIdxIt) {
			arc->disable();
		}

		LOG(INFO) << *arc;

		if(arc->type == ArcType::RAY || arc->secondNodeIdx == MAX) {return;}

		auto secondNode = &wf.nodes[arc->secondNodeIdx];
		auto arcs = &secondNode->arcs;

		/* remove reference to 'arcIdx' from node */
		if(!secondNode->removeArc(arcIdxIt)) {return;}

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
				return;
			}
		}
	}
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

		outfile << "f " << e.u+1;

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
