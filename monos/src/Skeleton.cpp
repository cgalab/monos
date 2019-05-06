
#include "Skeleton.h"


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
	LOG(INFO) << "START SINGLE MERGE STEP " << upperChainIndex << "/" << lowerChainIndex;
	/* we start the bisector from the source node from the "left" since the merge line is monotone */
	auto bis = wf.constructBisector(upperChainIndex,lowerChainIndex);
	if( wf.nodes[sourceNodeIdx].type != NodeType::TERMINAL && (bis.direction() != data.perpMonotonDir || bis.direction() != -data.perpMonotonDir) ) {
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

	/* debug */
	if(data.gui) {
		Edge visBis(bis.source(),bis.source()+10*bis.to_vector());
		if(!data.lines.empty()) {data.lines.pop_back();}
		data.lines.push_back(visBis);
	}

	std::vector<uint> arcs;
	Point newPoint = INFPOINT;
	bool  onUpperChain = true;

	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	findNextIntersectingArc(bis,arcs,onUpperChain,newPoint);

	if(!arcs.empty()) {
		LOG(INFO) << "After findNextIntersectingArc: arcs NOT empty!";
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
	} else {
		LOG(INFO) << "After findNextIntersectingArc: arcs ARE empty!";
	}

	return !EndOfBothChains();
}

/* finds the next arc(s) intersected by the bisector 'bis' that lie closest to 'sourceNode'
 * in respect to the 'monotonicityLine' */
void Skeleton::findNextIntersectingArc(const Ray& bis, std::vector<uint>& arcs, bool& setUpperChain, Point& newPoint) {
	assert(sourceNode != nullptr);

	/* new intersection point must be to the right of 'currentPoint' in respect to the monotonicity line */
	auto& currentPoint = sourceNode->point;
	bool success = false, localOnUpperChain;
	Point Pi = INFPOINT;
	MonotonePathTraversal* path;
	Arc *arc, *arc_u, *arc_l;

	LOG(INFO) << "findNextIntersectingArc start"; fflush(stdout);
	while(!EndOfBothChains() && !success) {
		arc_l = (EndOfLowerChain()) ? nullptr : wf.getArc(wf.lowerPath);
		arc_u = (EndOfUpperChain()) ? nullptr : wf.getArc(wf.upperPath);

		/* check which arc lies further to the left */
		if(arc_l == nullptr) {
			localOnUpperChain = true;
		} else if(arc_u == nullptr) {
			localOnUpperChain = false;
		} else {
			localOnUpperChain = (wf.isArcLeftOfArc(*arc_l,*arc_u)) ? false : true;
		}
		path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
		arc  = (localOnUpperChain) ? arc_u         : arc_l;

		std::cout << std::boolalpha<< "localOnUpperChain: " << localOnUpperChain << std::endl;
		std::cout << "BEFORE path: " << *path << std::endl;
		fflush(stdout);

		if(isValidArc(path->currentArcIdx) && do_intersect(bis,*arc)) {
			Pi = intersectRayArc(bis,*arc);
			LOG(INFO) << "Intersection found with " << path->currentArcIdx;
			if(data.monotoneSmaller(currentPoint,Pi)) {
				LOG(INFO) << "success";
				success = true;
			}
		}

		if(!success) {
			if(!wf.nextMonotoneArcOfPath(*path)) {
				if(localOnUpperChain) {
					upperChainIndex = nextUpperChainIndex(upperChainIndex);
					wf.initPathForEdge(true,upperChainIndex);
				} else {
					lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
					wf.initPathForEdge(false,lowerChainIndex);
				}
				LOG(WARNING) << "next chain index";
			}
		}

		std::cout << std::endl << "AFTER  path: " << *path << std::endl; fflush(stdout);
	}

	if(success) {
		/* we found an intersecting arc on 'localOnUpperChain', now we have to traverse
		 * the face on the opposite side until we reach the height of the intersecting
		 * arc, i.r.t. the monotonicity line
		 ***/
		path = (!localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
		Arc* arc;

		bool piReached = false;
		Point Pi_2 = INFPOINT;

		while(!EndOfChain(path->isUpperChain()) && !piReached) {
			std::cout << std::endl << "updateing 2nd  path: " << *path << std::endl; fflush(stdout);
			arc = wf.getArc(*path);

			if(isValidArc(path->currentArcIdx) && do_intersect(bis,*arc)) {
				Pi_2 = intersectRayArc(bis,*arc);
				LOG(INFO) << "Intersection found with " << path->currentArcIdx;
				if(data.monotoneSmaller(currentPoint,Pi_2)) {
					if(data.monotoneSmaller(Pi_2,Pi)) {
						LOG(INFO) << "success";
						piReached = true;
						Pi = Pi_2;
						localOnUpperChain = !localOnUpperChain;
					} else {
						piReached = true;
						path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
					}
				}
			}
			if(!piReached) {
				Point Pr = wf.nodes[wf.getRightmostNodeIdxOfArc(*arc)].point;
				// TODO: think about this, it is not correct
				if(data.monotoneSmaller(Pi,Pr)) {
					piReached = true;
					path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
				}
			}
			if(!piReached) {
				/* check input edge intersection first */
				Edge e = data.getEdge(path->edgeIdx);
				if(do_intersect(bis,e)) {
					LOG(INFO) << " intersecting input edge!";
					piReached = true;
					path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
				} else if(!wf.nextMonotoneArcOfPath(*path)) {
					/* iterate over path */
					if(path->isUpperChain()) {
						upperChainIndex = nextUpperChainIndex(upperChainIndex);
						wf.initPathForEdge(true,upperChainIndex);
					} else {
						lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
						wf.initPathForEdge(false,lowerChainIndex);
					}
					LOG(WARNING) << "next chain index";
				}
			}
		}

		if(!piReached) {
			path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
		}

		/* setting the 'newPoint' to the found intersection if 'success' */
		newPoint = Pi;
		arcs.push_back(path->currentArcIdx);
		setUpperChain = localOnUpperChain;
	} else {
		LOG(ERROR) << "NO INTERSECTION FOUND!!!";
		assert(false);
	}

	LOG(INFO) << "findNextIntersectingArc END"; fflush(stdout);
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
		if(arcIdx < MAX) {
			/* TODO: parallel-bisectors, direction unclear */
			LOG(INFO) << "update arc target " << arcIdx << " to end at " << newNodeIdx;
			updateArcTarget(arcIdx,edgeIdxA,newNodeIdx,p);
		}
	}
	return newNodeIdx;
}



void Skeleton::updateArcTarget(const uint& arcIdx, const uint& edgeIdx, const int& secondNodeIdx, const Point& edgeEndPoint) {
	auto arc = &wf.arcList[arcIdx];

	/* remove or disable rest of path that we broke? */
	/* we act in case the degree of the second node is <3, otherwise for >2 the path is still accessible from another route */
	if(arc->type == ArcType::NORMAL) {
		removePath(arcIdx, edgeIdx);
	}

	std::cout << "arc 10: " << wf.arcList[10] << std::endl;

	auto newNode = &wf.nodes[secondNodeIdx];
	if(arc->type == ArcType::RAY) {
		arc->type = ArcType::NORMAL;
		arc->edge = Edge(arc->ray.source(),edgeEndPoint);
		arc->ray = Ray();
	} else {
		assert(arc->isEdge());
		arc->edge = Edge(arc->edge.source(),edgeEndPoint);
	}

	arc->secondNodeIdx = secondNodeIdx;
	newNode->arcs.push_back(arcIdx);
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

		std::cout << " arc-start: " << arc->firstNodeIdx << " "; fflush(stdout);
		std::cout << " arc-endpoint: " << arc->secondNodeIdx << " "; fflush(stdout);

		auto secondNode = &wf.nodes[arc->secondNodeIdx];
		auto arcs = &secondNode->arcs;

		std::cout << " arcs:" << arcs->size() << " "; fflush(stdout);

		/* remove reference to 'arcIdx' from node */
		auto pos = std::find(arcs->begin(),arcs->end(),arcIdxIt);
		if(pos != arcs->end()) {
			arcs->erase(pos);
		} else {
			LOG(WARNING) << "Should not occur!";
			return false;
		}

		if(arcs->size() < 3) {
			/* degree two means that only one path is left */
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
