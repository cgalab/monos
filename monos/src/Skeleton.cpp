
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
	/* we start the bisector from the source node to the left since the merge line is 'x'-monotone */
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
			if(!EndOfChain()) {
				wf.initPathForEdge(true,upperChainIndex);
			}
		} else {
			lowerChainIndex = (modifiedArc->leftEdgeIdx != lowerChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			if(!EndOfChain()) {
				wf.initPathForEdge(false,lowerChainIndex);
			}
		}
		sourceNodeIdx = newNodeIdx;
		sourceNode = &wf.nodes[sourceNodeIdx];
	} else {
		LOG(INFO) << "After findNextIntersectingArc: arcs ARE empty!";
	}

	return upperChainIndex != wf.startUpperEdgeIdx || lowerChainIndex != wf.endLowerEdgeIdx;
}

bool Skeleton::EndOfChain() const {
	return upperChainIndex == wf.startUpperEdgeIdx || lowerChainIndex == wf.endLowerEdgeIdx;
}


/* finds the next arc(s) intersected by the bisector 'bis' that lie closest to 'sourceNode'
 * in respect to the 'monotonicityLine' */
void Skeleton::findNextIntersectingArc(const Ray& bis, std::vector<uint>& arcs, bool& setUpperChain, Point& newPoint) {
	assert(sourceNode != nullptr);

	/* new intersection point must be to the right of 'currentPoint' in reps. to the monotonicity line */
	auto& currentPoint = sourceNode->point;
	bool success = false, localOnUpperChain;
	Point Pi = INFPOINT;
	MonotonePathTraversal* path;
	Arc *arc, *arc_u, *arc_l;

	LOG(INFO) << "findNextIntersectingArc start"; fflush(stdout);
	while(!EndOfChain() && !success) {
		arc_l = wf.getArc(wf.lowerPath);
		arc_u = wf.getArc(wf.upperPath);

		/* check arcs in respect to source node */
//		updatePath(sourceNode->point,wf.lowerPath);
//		updatePath(sourceNode->point,wf.upperPath);

		/* check which arc lies futher to the left */
		localOnUpperChain = (wf.isArcLeftOfArc(*arc_l,*arc_u)) ? false : true;

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
		std::cout << std::endl << "updateing 2nd  path: " << *path << std::endl; fflush(stdout);

		while(!EndOfChain() && !piReached) {
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

		/* setting the 'newPoint' to the found intersection if 'success' */
		newPoint = Pi;
		arcs.push_back(path->currentArcIdx);
		setUpperChain = localOnUpperChain;
	}

	LOG(INFO) << "findNextIntersectingArc END"; fflush(stdout);
}

void Skeleton::updatePath(const Point &p, MonotonePathTraversal& path) {
	auto arc = wf.getArc(path);
	while(wf.isArcLeftOfPoint(*arc,p) && wf.nextMonotoneArcOfPath(path))
	;
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
	if(arc->type == ArcType::NORMAL) {
		auto oldNode = &wf.nodes[arc->secondNodeIdx];
		uint nextArcIdx = arcIdx;
		while(nextArcOnPath(nextArcIdx, edgeIdx, nextArcIdx)) {
			if(nextArcIdx != arcIdx) {
				wf.arcList[nextArcIdx].disable();
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
		assert(arc->isEdge());
		arc->edge = Edge(arc->edge.source(),edgeEndPoint);
	}

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

				if(	 arc->secondNodeIdx == nextArc->firstNodeIdx
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
