
#include "Skeleton.h"


/* walk along the arcs adjacent to 'edgeIdx' until we find an intersection with 'ray'
 * we walk along both face paths */
bool Skeleton::findRayFaceIntersection(const uint& edgeIdx, const Ray& ray, const bool upperChain, uint& arcIdx, Point& intersection) {
	auto edge 	   = data.e(edgeIdx);
	bool success   = false;

	/* every face consists of 'at most' two paths */
	uint pathCount = 2;
	Node* node;

	/* as the merge s
	 * starts from the relative 'leftmost' part we use the path closest to that side first */
	/* we have to consider that we might end up following the 'leftmost' vertex, i.e., merge vertex */
	if(upperChain) {
		if(!isMergeStartEndNodeIdx(edge[1])) {
			node = &wf.getTerminalNodeForVertex( edge[1] );
		} else {
			node = &wf.getTerminalNodeForVertex( edge[0] );
		}
	} else {
		if(!isMergeStartEndNodeIdx(edge[0])) {
			node = &wf.getTerminalNodeForVertex( edge[0] );
		} else {
			node = &wf.getTerminalNodeForVertex( edge[1] );
		}
	}

	if(isMergeStartEndNodeIdx(edge[0]) || isMergeStartEndNodeIdx(edge[1])) {--pathCount;}

	std::cout << " fPRI(" << edge[0] << "," << edge[1] << ": "<< node->arcs.size() << ") " << std::endl; fflush(stdout);

	while(pathCount > 0 && !success) {
		if(node->arcs.size() > 0) {
			auto arcItIdx = node->arcs.front();
			do {
				std::cout << " path-" << pathCount << " "; fflush(stdout);
				auto arc = &wf.arcList[arcItIdx];

				std::cout << " " << arc->firstNodeIdx << "->" << arc->secondNodeIdx << " ";fflush(stdout);

				/* check that we do not find our source node as a new intersection  */
				if(arc->secondNodeIdx < startIdxMergeNodes || arc->secondNodeIdx == MAX) {
//				if(arc->firstNodeIdx < startIdxMergeNodes) {
//				if(arc->secondNodeIdx < sourceNodeIdx) {
					/* find an intersection */
					auto p = intersectArcRay(*arc,ray);
					if(p != INFPOINT) {
						LOG(INFO) << "YES!";
						success = true;
						intersection = p;
						arcIdx = arcItIdx;
					}
				}
			} while( !success && nextArcOnPath(arcItIdx,edgeIdx, arcItIdx) );
		}
		/* the second path is set, as every face consists of two paths */
		node = &wf.getTerminalNodeForVertex( (upperChain) ? edge[0] : edge[1] );
		--pathCount;
	}
	if(!success) {
		intersection = INFPOINT;
		LOG(INFO) << "NO intersection at face " << edgeIdx << ".";
		Edge e(data.getEdge(edgeIdx));
		auto p = intersectElements(e,ray);
		if(p != INFPOINT) {
			LOG(INFO) << "YES! intersects Edge ";
			success = true;
			intersection = p;
			arcIdx = INFINITY;
		}
	}
	return success;
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
	wf.addArc(mergeEndNodeIdx(),sourceNodeIdx,lowerChainIndex,upperChainIndex);
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
	/* we start the bisector from the source node to the left since the merge line is 'x'-monotone */
	auto bis = wf.constructBisector(upperChainIndex,lowerChainIndex);
	if( wf.nodes[sourceNodeIdx].type != NodeType::TERMINAL && !CGAL::parallel( bis.supporting_line() , data.monotonicityLine.perpendicular(data.monotonicityLine.point()) ) ) {
		auto lastArcNodeIdx = (wf.getLastArc()->firstNodeIdx != sourceNodeIdx) ? wf.getLastArc()->firstNodeIdx : wf.getLastArc()->secondNodeIdx;
		auto pointOnLastArc = wf.nodes[lastArcNodeIdx].point;
		auto pointOnLastArcProjected = bis.supporting_line().projection(pointOnLastArc);

		bis = Ray(sourceNode->point, sourceNode->point - pointOnLastArcProjected);

	} else if(wf.nodes[sourceNodeIdx].type != NodeType::TERMINAL) {
		/* bisector is vertical in respect to monotonicity line */
		Point P = intersectElements(bis,data.bbox.bottom);
		if (P == INFPOINT) {
			P = intersectElements(bis,data.bbox.right);
		}
		if (P == INFPOINT) {
			P = intersectElements(bis,data.bbox.top);
		}
		if (P == INFPOINT) {
			P = intersectElements(bis,data.bbox.left);
		}
		bis = Ray(P,-bis.direction());

		LOG(INFO) << "bisector is vertical!";
	}

	std::cout << " bis: u/l " << upperChainIndex << "/" << lowerChainIndex << " - " << bis;
	fflush(stdout);
	Edge eA = data.getEdge(upperChainIndex);
	Edge eB = data.getEdge(lowerChainIndex);

	std::cout << " Points: " << eA.source() << "," << eA.target();
	std::cout << " / " << eB.source() << "," << eB.target() << std::endl;

	/* identify arcs and intersection points */
	uint  upperArcIdx, lowerArcIdx;
	Point pU = INFPOINT, pL = INFPOINT;

	std::vector<uint> arcs;
	Point newPoint = INFPOINT;
	bool  onUpperChain = true;

	std::cout << std::endl << "UPPER u/l " << upperChainIndex << "/" << lowerChainIndex << ":"; fflush(stdout);

	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	findNextIntersectingArc(bis,arcs,onUpperChain,newPoint);

	if(!arcs.empty()) {
		newNodeIdx = handleMerge(arcs,upperChainIndex,lowerChainIndex,newPoint,bis);

		Arc* modifiedArc = &wf.arcList[arcs.front()];

		if(onUpperChain) {
			upperChainIndex = (modifiedArc->leftEdgeIdx != upperChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			wf.initPathForEdge(true,upperChainIndex);
		} else {
			lowerChainIndex = (modifiedArc->leftEdgeIdx != lowerChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			wf.initPathForEdge(false,lowerChainIndex);
		}
		sourceNodeIdx = newNodeIdx;
		sourceNode = &wf.nodes[sourceNodeIdx];
	}

//	if( (upperChainIndex+1 != wf.startUpperEdgeIdx || lowerChainIndex != wf.endLowerEdgeIdx+1) ) {
//		auto distNodePU = CGAL::squared_distance(sourceNode->point, pU);
//		auto distNodePL = CGAL::squared_distance(sourceNode->point, pL);
//
//		if( distNodePU < distNodePL || pL == INFPOINT) {
//			/* upper chain intersection is closer */
//			std::cout << " -u- (" << upperArcIdx << ") "; fflush(stdout);
//			std::vector<uint> arcInices{upperArcIdx};
//			newNodeIdx = handleMerge(arcInices,upperChainIndex,lowerChainIndex,pU,bis);
//
//			auto upperArc = &wf.arcList[upperArcIdx];
//			upperChainIndex = upperArc->leftEdgeIdx;
//		} else if (distNodePU > distNodePL || pU == INFPOINT) {
//			/* lower chain is closer */
//			std::cout << " -l- (" << lowerArcIdx << ") "; fflush(stdout);
//			std::vector<uint> arcInices{lowerArcIdx};
//			newNodeIdx = handleMerge(arcInices,upperChainIndex,lowerChainIndex,pL,bis);
//
//			auto lowerArc = &wf.arcList[lowerArcIdx];
//			lowerChainIndex = lowerArc->rightEdgeIdx;
//		} else {
//			assert(pU == pL);
//
//			/* multi event */
//			std::cout << " -m(TODO arcs)- "; fflush(stdout);
//			std::vector<uint> arcInices{upperArcIdx,lowerArcIdx};
//			newNodeIdx = handleMerge(arcInices,upperChainIndex,lowerChainIndex,pU,bis);
//
//			upperChainIndex = nextUpperChainIndex(upperChainIndex);
//			lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
//		}
//
//		/* TODO: node is sorted exept for new arc, add it on the right place! */
//		//sourceNode->sort(wf.arcList);
//
//		sourceNodeIdx = newNodeIdx;;
//		sourceNode    = &wf.nodes[sourceNodeIdx];
//	}
//	if(upperChainIndex+1 == wf.startUpperEdgeIdx && lowerChainIndex != wf.endLowerEdgeIdx+1) {
//		--upperChainIndex;
//	}
//	if(upperChainIndex+1 != wf.startUpperEdgeIdx && lowerChainIndex == wf.endLowerEdgeIdx+1) {
//		--lowerChainIndex;
//	}

	return upperChainIndex != wf.startUpperEdgeIdx || lowerChainIndex != wf.endLowerEdgeIdx;
}

bool Skeleton::EndOfChain() const {
	return upperChainIndex+1 == wf.startUpperEdgeIdx || lowerChainIndex == wf.endLowerEdgeIdx+1;
}

void Skeleton::findNextIntersectingArc(const Ray& bis, std::vector<uint>& arcs, bool& upperChain, Point& newPoint) {
	assert(sourceNode != nullptr);

	/* new intersection point must be to the right of 'currentPoint' in reps. to the monotonicity line */
	auto& currentPoint = sourceNode->point;
	bool success = false, onUpperChain;
	Point Pi_u = INFPOINT, Pi_l = INFPOINT;

	Arc* Arc_u;
	Arc* Arc_l;
	std::cout << " go " << std::endl; fflush(stdout);
	while(!EndOfChain() && !success) {
		Arc_l = wf.getArc(wf.lowerPath);
		Arc_u = wf.getArc(wf.upperPath);

		/* check which arc lies futher to the left */
		onUpperChain = (wf.isArcLeftOfArc(*Arc_l,*Arc_u)) ? false : true;

		std::cout << std::boolalpha<< " upperChain: " << onUpperChain << std::endl;
		std::cout << " BEFORE paths u: " << wf.upperPath << " // l: " << wf.lowerPath;
		fflush(stdout);

		if(onUpperChain) {
			if(isValidArc(wf.upperPath.currentArcIdx) && do_intersect(bis,*Arc_u)) {
				Pi_u = intersectRayArc(bis,*Arc_u);
				if(data.monotoneSmaller(currentPoint,Pi_u)) {
					success = true;
				}
			}
			if(!success && !wf.nextMonotoneArcOfPath(wf.upperPath)) {
				upperChainIndex = nextUpperChainIndex(upperChainIndex);
				wf.initPathForEdge(true,upperChainIndex);
				LOG(WARNING) << "Check findNextIntersectingArc (upper)";
			}

		} else {
			std::cout << " a " << bis << " " << std::endl;
			if( isValidArc(wf.lowerPath.currentArcIdx) && do_intersect(bis,*Arc_l) ) {
				std::cout << " 1 " << Arc_l->edge << " " << std::endl;
				Pi_l = intersectRayArc(bis,*Arc_l);
				if(data.monotoneSmaller(currentPoint,Pi_l)) {
					std::cout << " s " << std::endl;
					success = true;
				}
			}
			std::cout << " b " << std::endl;
			if(!success && !wf.nextMonotoneArcOfPath(wf.lowerPath)) {
				std::cout << " 2 " << std::endl;
				lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
				wf.initPathForEdge(false,lowerChainIndex);
				LOG(WARNING) << "Check findNextIntersectingArc (lower)";
			}

		}

		std::cout << std::endl << "uc:" << upperChainIndex << " "; fflush(stdout);
		std::cout << " AFTER paths u: " << wf.upperPath << " // l: " << wf.lowerPath;
		fflush(stdout);
	}

	/* setting the 'newPoint' to the found intersection if 'success' */
	if(success) {
		if(onUpperChain) {
			newPoint = Pi_u;
			arcs.push_back(wf.upperPath.currentArcIdx);
		} else {
			newPoint = Pi_l;
			arcs.push_back(wf.lowerPath.currentArcIdx);
		}
		upperChain = onUpperChain;
	}

//	while(upperChainIndex+1 != wf.startUpperEdgeIdx
//			&& !findRayFaceIntersection(upperChainIndex,bis,true, upperArcIdx, pU)) {
//		upperChainIndex = nextUpperChainIndex(upperChainIndex);
//		std::cout << std::endl << "uc:" << upperChainIndex << " "; fflush(stdout);
//	}

	std::cout << std::endl << "LOWER u/l " << upperChainIndex << "/" << lowerChainIndex << ":"; fflush(stdout);

//	while(lowerChainIndex != wf.endLowerEdgeIdx+1
//			&& !findRayFaceIntersection(lowerChainIndex,bis,false, lowerArcIdx, pL)) {
//		lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
//		std::cout << " lc:" << lowerChainIndex << " "; fflush(stdout);
//	}
//
//	std::cout << "." << upperChainIndex << " " << lowerChainIndex
//			  << " pU: " << pU << " pL: " << pL; fflush(stdout);

}

uint Skeleton::handleMerge(const std::vector<uint>& arcIndices, const uint& edgeIdxA, const uint& edgeIdxB, const Point& p, const Ray& bis) {
	auto sourceNode = &wf.nodes[sourceNodeIdx];

	auto distA = data.normalDistance(edgeIdxA,sourceNode->point);
	auto distB = data.normalDistance(edgeIdxA,p);

	auto newNodeIdx = wf.addNode(p,distB);
	auto newNode    = &wf.nodes[newNodeIdx];

	/* distinguish in which direction the ray points and add the arc accordingly */
	uint newArcIdx = 0;
	if(distA < distB) {
		newArcIdx  = wf.addArc(sourceNodeIdx,newNodeIdx,edgeIdxA,edgeIdxB);
	} else {
		newArcIdx  = wf.addArc(newNodeIdx,sourceNodeIdx,edgeIdxB,edgeIdxA);
	}

	/* update the targets of the relevant arcs */
	for(auto arcIdx : arcIndices) {
		if(arcIdx < INFINITY) {
			/* TODO: parallel-bisectors, direction unclear */
			updateArcTarget(arcIdx,newNodeIdx,p);
		}
	}
	return newNodeIdx;
}



void Skeleton::updateArcTarget(const uint& arcIdx, const int& secondNodeIdx, const Point& edgeEndPoint) {
	auto arc = &wf.arcList[arcIdx];

	/* remove or disable rest of path that we broke? */
	if(arc->type == ArcType::NORMAL) {
		auto oldNode = &wf.nodes[arc->secondNodeIdx];
		uint nextArcIdx = arcIdx;
		while(nextArcOnPath(nextArcIdx, arc->leftEdgeIdx, nextArcIdx)) {
			if(nextArcIdx != arcIdx) {
				wf.arcList[nextArcIdx].disable();
			}
		}
		nextArcIdx = arcIdx;
		while(nextArcOnPath(nextArcIdx, arc->rightEdgeIdx, nextArcIdx)) {
			if(nextArcIdx != arcIdx) {
				wf.arcList[nextArcIdx].disable();
				//wf.nodes[wf.arcList[nextArcIdx].secondNodeIdx].disable();
			}
		}
		oldNode->disable(); //arcs.clear();
	}

	auto newNode = &wf.nodes[secondNodeIdx];
	if(arc->type == ArcType::RAY) {
		arc->type = ArcType::NORMAL;
		arc->edge = Edge(arc->ray.source(),edgeEndPoint);
		arc->ray = Ray();
	} else {
		arc->edge = Edge(arc->edge.source(),edgeEndPoint);
	}

	std::cout << " arc: " << arc->firstNodeIdx << "->" << arc->secondNodeIdx << " (" << secondNodeIdx << ") ";
	fflush(stdout);


	arc->secondNodeIdx = secondNodeIdx;
	newNode->arcs.push_back(arcIdx);
}

bool Skeleton::nextArcOnPath(const uint& arcIdx, const uint& edgeIdx, uint& nextArcIdx) const {
	auto arc  = &wf.arcList[arcIdx];

//	if(arc->type == ArcType::RAY || arc->type == ArcType::DISABLED) {return false;}
	if(arc->type == ArcType::RAY || arc->secondNodeIdx == MAX) {return false;}

	std::cout << " type: ";
	switch(arc->type) {
	case ArcType::RAY: std::cout << " RAY "; break;
	case ArcType::NORMAL: std::cout << " NORMAL "; break;
	case ArcType::DISABLED: std::cout << " DISAB "; break;
	}

	std::cout << " arc-start: " << arc->firstNodeIdx << " "; fflush(stdout);
	std::cout << " arc-endpoint: " << arc->secondNodeIdx << " "; fflush(stdout);


	auto arcs = &wf.nodes[arc->secondNodeIdx].arcs;

	std::cout << " arcs:" << arcs->size() << " "; fflush(stdout);

	if(arcs->size() < 20) {

		for(auto a : *arcs) {
			if(a != arcIdx) {
				auto nextArc = &wf.arcList[a];

				if( arc->type != ArcType::RAY
					&&  arc->secondNodeIdx == nextArc->firstNodeIdx
					&& (nextArc->leftEdgeIdx == arc->leftEdgeIdx || nextArc->rightEdgeIdx == arc->rightEdgeIdx )
				) {
					/* set 'return' value */
					nextArcIdx = a;

					LOG(INFO) << " is adjacent! ";
					return true;
				}
			}
		}

		LOG(WARNING) << " no more arc on path! ";
		return false;

	} else {

		/* TODO: fix this: */
		auto it = std::lower_bound(arcs->begin(),arcs->end(),arcIdx,ArcCmp(wf.arcList));
		assert(it != arcs->end());

		auto nextArc = &wf.arcList[*(++it)];
		if(nextArc->leftEdgeIdx == edgeIdx || nextArc->rightEdgeIdx == edgeIdx) {
			nextArcIdx = *it;
			return true;
		} else {
			--it; --it;
			nextArc = &wf.arcList[*(it)];
			if(nextArc->leftEdgeIdx == edgeIdx || nextArc->rightEdgeIdx == edgeIdx) {
				nextArcIdx = *it;
				return true;
			} else {
				LOG(WARNING) << " OH NO! SOME SORTING FAILED!! ";
				return false;
			}
		}
	}
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
		double xt, yt, zt, xm, ym, zm;
		getNormalizer(data.bbox,xt,xm,yt,ym,zt,zm);

		std::ofstream outfile (cfg.outputFileName,std::ofstream::binary);
		outfile << "# OBJ-File autogenerated by monos from file ("
				<< cfg.fileName << ") - "
				<< currentTimeStamp() <<  std::endl;

		for(auto n : wf.nodes) {
			double x = (n.point.x().doubleValue() - xt) * xm;
			double y = (n.point.y().doubleValue() - yt) * ym;
			outfile << "v " << x << " " << y << " 0.0"<< std::endl;
		}

		for(uint i = 0; i < wf.arcList.size(); ++i) {
			auto a = &wf.arcList[i];
			/* +1 is the standard OBJ offset for references */
			if(a->type == ArcType::NORMAL && wf.isArcInSkeleton(i)) {
				outfile << "l " << a->firstNodeIdx+1 << " " << a->secondNodeIdx+1 << std::endl;
			} else {
				if(a->firstNodeIdx < wf.nodes.size()) {
					auto nodeA = wf.nodes[a->firstNodeIdx];
					nodeA.arcs.clear();
				}
				if(a->secondNodeIdx < wf.nodes.size()) {
					auto nodeB = wf.nodes[a->secondNodeIdx];
					nodeB.arcs.clear();
				}
			}
		}

		outfile.close();
	}
}
