
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
	LOG(INFO) << "-- START SINGLE MERGE STEP " << upperChainIndex << "/" << lowerChainIndex;
	fflush(stdout);

	/* we start the bisector from the source node from the "left" since the merge line is monotone */
	Bisector bisGeneral = wf.constructBisector(upperChainIndex,lowerChainIndex);
	std::cout << "-"; fflush(stdout);
	std::cout << bisGeneral.direction(); fflush(stdout);
	Bisector bis = wf.getBisectorWRTMonotonicityLine(bisGeneral);
	std::cout << "/"; fflush(stdout);
	std::cout << "|"; fflush(stdout);
	std::cout << bis.direction(); fflush(stdout);
	std::cout << "p: "; fflush(stdout);
	std::cout << sourceNode->point; fflush(stdout);
	std::cout << ", dir: "; fflush(stdout);
	std::cout << bis.direction(); fflush(stdout);
	if(bis.isLine() && !bis.perpendicular) {
//		auto d = bis.direction();
//		Ray r = Ray(sourceNode->point, d);
//		bis = Bisector(r);
//		std::cout << " set ray, from new source: "; fflush(stdout);
//		std::cout << *r; fflush(stdout);
//		std::cout << " dir: "; fflush(stdout);
//		std::cout << d; fflush(stdout);
//		std::cout << " ray p0: "; fflush(stdout);
//		std::cout << r->source(); fflush(stdout);
//		std::cout << " ray p1: "; fflush(stdout);
//		std::cout << r->point(1); fflush(stdout);
//		std::cout << " ray-dir: "; fflush(stdout);
//		std::cout << std::boolalpha << bis.line.is_horizontal();
//		std::cout << r->direction(); fflush(stdout);
		bis.newSource(sourceNode->point);
//		bis.setRay(*r);
	} else {
		std::cout << " new source: "; fflush(stdout);
		std::cout << sourceNode->point; fflush(stdout);
		bis.newSource(sourceNode->point);
	}

	std::cout << std::endl << "-- "; fflush(stdout);
	LOG(INFO) << "Bisector: " << bis; //.to_vector(); fflush(stdout);
	std::cout << "-- "; fflush(stdout);
	LOG(INFO) << "Bisector-dir: " << bis.direction(); //.to_vector(); fflush(stdout);

	/* visualize next bisector via dashed line-segment */
//	if(data.gui) {
//		Point A(sourceNode->point);
//		Point B(A + (10*bis.to_vector()));
//		Edge visBis( Point(A.x().doubleValue(),A.y().doubleValue()) ,
//			      	 Point(B.x().doubleValue(),B.y().doubleValue()) );
//		if(!data.lines.empty()) {data.lines.pop_back();}
//		data.lines.push_back(visBis);
//	}

	/* setup intersection call */
	std::vector<uint> arcs;
	Point newPoint = INFPOINT;
	bool  onUpperChain = true;

	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	findNextIntersectingArc(bis,arcs,onUpperChain,newPoint);

	if(!arcs.empty()) {
		LOG(INFO) << "After findNextIntersectingArc: arcs NOT empty!" << newPoint;
		newNodeIdx = handleMerge(arcs,upperChainIndex,lowerChainIndex,newPoint,bis);

		Arc* modifiedArc = &wf.arcList[arcs.front()];
		if(onUpperChain) {
			upperChainIndex = (modifiedArc->leftEdgeIdx != upperChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			wf.initPathForEdge(true,upperChainIndex);
		} else {
			lowerChainIndex = (modifiedArc->leftEdgeIdx != lowerChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
			wf.initPathForEdge(false,lowerChainIndex);
		}

		LOG(INFO) << "changing idx from old: " << sourceNodeIdx << " to " << newNodeIdx;

		sourceNodeIdx = newNodeIdx;
		sourceNode = &wf.nodes[sourceNodeIdx];
	} else {
		LOG(INFO) << "After findNextIntersectingArc: arcs ARE empty!";
	}

	std::cout << std::endl;
	LOG(INFO) << "############################################## END STEP ###########################";
	std::cout << std::endl;

	return !EndOfBothChains();
}

/* finds the next arc(s) intersected by the bisector 'bis' that lie closest to 'sourceNode'
 * in respect to the 'monotonicityLine' */
void Skeleton::findNextIntersectingArc(Bisector& bis, std::vector<uint>& arcs, bool& setUpperChain, Point& newPoint) {
	assert(sourceNode != nullptr);

	/* new intersection point must be to the right of 'currentPoint' in respect to the monotonicity line */
	auto& currentPoint = sourceNode->point;
	bool success = false, localOnUpperChain;
	Point Pi = INFPOINT;
	MonotonePathTraversal* path;
	Arc *arc, *arc_u, *arc_l;

	bool bisOnPositiveSide = true, bisUpdateOnce = false;
	if(bis.perpendicular && bis.isRay()) {
		auto vb   = bis.to_vector();
		Point pML = data.monotonicityLine.point(0) + vb;
		bisOnPositiveSide = data.monotonicityLine.has_on_positive_side(pML);
		bisUpdateOnce 	  = true;
		LOG(INFO) << "Bisector is perpendicular and a ray!";
	}

	LOG(INFO) << "findNextIntersectingArc start"; fflush(stdout);
	while(!EndOfBothChains() && !success) {
		std::cout << ".";
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

		LOG(INFO) << "Test if perp and update clause!";
		if(bis.perpendicular && bisUpdateOnce) {
			if((bisOnPositiveSide && localOnUpperChain) || (!bisOnPositiveSide && !localOnUpperChain) ) {
				LOG(INFO) << "findNextIntersectingArc() -- change direction";
				bis.changeDirection();
			}
			bisUpdateOnce = false;
		}

		LOG(INFO) << "testing arc: " << *arc;

		if(isValidArc(path->currentArcIdx)) { //&& do_intersect(bis,*arc)) {
			Pi = intersectBisectorArc(bis,*arc);
			if(Pi != INFPOINT) {
				LOG(INFO) << "Intersection found with " << path->currentArcIdx; fflush(stdout);
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
				piReached = true;

				Pi_2 = intersectBisectorArc(bis,*arc);
				LOG(INFO) << "Intersection found with " << path->currentArcIdx;

				bool choosePi;

				/* we found two points Pi and Pi_2, one on each chain */
				if(bis.perpendicular) {
					auto dPi   = normalDistance(data.monotonicityLine,Pi);
					auto dPi_2 = normalDistance(data.monotonicityLine,Pi_2);
					if(dPi < dPi_2) {
						choosePi = true;
					} else {
						choosePi = false;
					}
				} else {
					if(data.monotoneSmaller(Pi,Pi_2)) {
						choosePi = true;
					} else {
						choosePi = false;
					}
				}

				if(choosePi) {
					LOG(INFO) << "success (Pi)";
					path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
				} else {
					LOG(INFO) << "success (Pi_2)";
					Pi = Pi_2;
					localOnUpperChain = !localOnUpperChain;
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
	}

	LOG(INFO) << "findNextIntersectingArc END"; fflush(stdout);
}

uint Skeleton::handleMerge(const std::vector<uint>& arcIndices, const uint& edgeIdxA, const uint& edgeIdxB, const Point& p, const Bisector& bis) {
	auto sourceNode = &wf.nodes[sourceNodeIdx];

	auto distA = data.normalDistance(edgeIdxA,sourceNode->point);
	auto distB = data.normalDistance(edgeIdxA,p);

	auto newNodeIdx = wf.addNode(p,distB);
	auto newNode    = &wf.nodes[newNodeIdx];

	LOG(INFO) << " NEW IDX: " << newNodeIdx << "/" << wf.nodes.size();

	/* distinguish in which direction the ray points and add the arc accordingly */
	uint newArcIdx = 0;
	if(bis.isLine() || distA < distB) {
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
