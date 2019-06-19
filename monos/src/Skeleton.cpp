
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
	Bisector bis = wf.getBisectorWRTMonotonicityLine(bisGeneral);
	bis.newSource(sourceNode->point);

//	/* identify possible ghost arc scenario on start */
//	if(data.isEdgesCollinear(upperChainIndex,lowerChainIndex)) {
//		LOG(WARNING) << "Collienar input edges " << upperChainIndex << ", " << lowerChainIndex;
//	}

	std::cout << std::endl << "-- "; fflush(stdout);
	LOG(INFO) << "Bisector: " << bis;
	std::cout << "-- "; fflush(stdout);
	LOG(INFO) << "Bisector-dir: " << bis.direction(); fflush(stdout);

	/* visualize next bisector via dashed line-segment */
	if(data.gui) {
		Edge visBis(sourceNode->point,sourceNode->point+(5*bis.to_vector()) );
		if(!data.lines.empty()) {data.lines.pop_back();}
		data.lines.push_back(visBis);
	}

	/* setup intersection call */
	std::vector<uint> arcs;
	Point newPoint = INFPOINT;
	bool  onUpperChain = true;

	/* obtain the arcIdx and newPoint for the next bis arc intersection */
	findNextIntersectingArc(bis,arcs,onUpperChain,newPoint);

	if(!arcs.empty()) {
		LOG(INFO) << "After findNextIntersectingArc: arcs NOT empty!" << newPoint;
		newNodeIdx = handleMerge(arcs,upperChainIndex,lowerChainIndex,newPoint,bis);

		if(arcs.size() == 1) {
			Arc* modifiedArc = &wf.arcList[arcs.front()];
			if(onUpperChain) {
				upperChainIndex = (modifiedArc->leftEdgeIdx != upperChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
				wf.initPathForEdge(true,upperChainIndex);
			} else {
				lowerChainIndex = (modifiedArc->leftEdgeIdx != lowerChainIndex) ? modifiedArc->leftEdgeIdx : modifiedArc->rightEdgeIdx;
				wf.initPathForEdge(false,lowerChainIndex);
			}
		} else {
			assert(arcs.size() == 2);
			LOG(INFO) << "arcs " << arcs.front() << " and " << arcs.back() << " returned!";
			Arc* modifiedArcU = &wf.arcList[arcs.front()];
			upperChainIndex = (modifiedArcU->leftEdgeIdx != upperChainIndex) ? modifiedArcU->leftEdgeIdx : modifiedArcU->rightEdgeIdx;
			wf.initPathForEdge(true,upperChainIndex);

			Arc* modifiedArcL = &wf.arcList[arcs.back()];
			lowerChainIndex = (modifiedArcL->leftEdgeIdx != lowerChainIndex) ? modifiedArcL->leftEdgeIdx : modifiedArcL->rightEdgeIdx;
			wf.initPathForEdge(false,lowerChainIndex);
		}

		LOG(INFO) << "changing idx from old: " << sourceNodeIdx << " to " << newNodeIdx;

		sourceNodeIdx = newNodeIdx;
		sourceNode = &wf.nodes[sourceNodeIdx];

		if(addGhostNode) {
			sourceNode->setGhost(true);
			addGhostNode = false;
			LOG(INFO) << "SingleMergeStep: setting sourceNode to be a ghost node!";
		}
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
	Point Pi = INFPOINT, Pi_2 = INFPOINT;
	MonotonePathTraversal* path;
	MonotonePathTraversal pathBackupLower, pathBackupUpper;
	Arc *arc, *arc_u, *arc_l;

	bool bisOnPositiveSide = true, bisUpdateOnce = false;
	if(bis.isParallel() && bis.isRay()) {
		auto vb   = bis.to_vector();
		Point pML = data.monotonicityLine.point(0) + vb;
		bisOnPositiveSide = data.monotonicityLine.has_on_positive_side(pML);
		bisUpdateOnce 	  = true;
		LOG(INFO) << "Bisector is perpendicular and a ray!";
	}

	/* while iterate we may iterate one arc to far, this is an easy way to step back */
	pathBackupLower = wf.lowerPath;
	pathBackupUpper = wf.upperPath;

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

		LOG(INFO) << "Test if perp and update clause!";
		if(bis.isParallel() && bisUpdateOnce) {
			if((bisOnPositiveSide && localOnUpperChain) || (!bisOnPositiveSide && !localOnUpperChain) ) {
				LOG(INFO) << "findNextIntersectingArc() -- change direction";
				bis.changeDirection();
			}
			bisUpdateOnce = false;
		}

		LOG(INFO) << "intersect arc: " << *arc << ", and bisector " << bis; fflush(stdout);

		/* if we have a sourcenode that is a ghost node we handle the intersection here */
		if(handleSourceGhostNode(bis,arcs,newPoint)) {
			return;
		}

		/* detect and handle possible ghost vertex */
		Pi = handleGhostVertex(*path,*arc,bis);

		/* classical intersection detection on current paths arc */
		if(Pi == INFPOINT && isValidArc(path->currentArcIdx)) {
			auto lRef = bis.supporting_line();
			if(arc->isEdge()) {
				Point a = arc->edge.point(0);
				Point b = arc->edge.point(1);

				if( (lRef.has_on_positive_side(a) && lRef.has_on_positive_side(b)) ||
				    (lRef.has_on_negative_side(a) && lRef.has_on_negative_side(b))
				) {
					// no intersection
				} else {
					Pi = intersectBisectorArc(bis,*arc);
				}
			} else {
				Pi = intersectBisectorArc(bis,*arc);
			}
		}

		if(Pi != INFPOINT) {
			LOG(INFO) << "Intersection found with " << path->currentArcIdx; fflush(stdout);
			success = true;
		}

		if(!success) {
			/* while iterate we may iterate one arc to far, this is an easy way to step back */
			pathBackupLower = wf.lowerPath;
			pathBackupUpper = wf.upperPath;

			if(!wf.nextMonotoneArcOfPath(*path)) {
				if(localOnUpperChain) {
					upperChainIndex = nextUpperChainIndex(upperChainIndex);
					wf.initPathForEdge(true,upperChainIndex);
				} else {
					lowerChainIndex = nextLowerChainIndex(lowerChainIndex);
					wf.initPathForEdge(false,lowerChainIndex);
				}
				LOG(WARNING) << "next chain index";fflush(stdout);
			}
		}

		std::cout << std::endl << std::boolalpha << "AFTER success: " << success << ", path: " << *path << std::endl; fflush(stdout);
	}

	if(success) {
		/* we found an intersecting arc on 'localOnUpperChain', now we have to traverse
		 * the face on the opposite side until we reach the height of the intersecting
		 * arc, i.r.t. the monotonicity line
		 ***/

		path = (!localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
		Arc* arc = wf.getArc(*path);

		/* check for possible upcoming ghost arc */
		if(!EndOfChain(path->isUpperChain()) && hasCollinearEdges(*arc_u,*arc_l)) {
			LOG(INFO) << "parallel input edges, TODO: deal with weights";
			if(wf.isArcPerpendicular((*arc_u)) || wf.isArcPerpendicular((*arc_u))) {
				LOG(INFO) << "...and vertical arc(s).";
			}
		} else {
			/* skip if edges are collinear  */
			/* check and reset if we walked to far on the other chain */
			auto pathBackup = (!localOnUpperChain) ? pathBackupUpper : pathBackupLower;
			CheckAndResetPath(path, pathBackup, Pi);
		}

		bool piReached = false;
		Pi_2 = INFPOINT;

		while(!EndOfChain(path->isUpperChain()) && !piReached) {
			std::cout << std::endl << "updating 2nd path: " << *path << std::endl; fflush(stdout);
			arc = wf.getArc(*path);
			std::cout << "intersect " << *arc << ", and bis: " << bis << std::endl; fflush(stdout);

			/* classical intersection detection on current paths arc */
			if(isValidArc(path->currentArcIdx)) {

				/* if we have a sourcenode that is a ghost node we handle the intersection here */
				if(handleSourceGhostNode(bis,arcs,newPoint)) {
					return;
				}
				/* detect and handle possible ghost vertex */
				Pi_2 = handleGhostVertex(*path,*arc,bis);

				if(Pi_2 == INFPOINT) {
					auto lRef = bis.supporting_line();
					if(arc->isEdge()) {
						Point a = arc->edge.point(0);
						Point b = arc->edge.point(1);

						if( (lRef.has_on_positive_side(a) && lRef.has_on_positive_side(b)) ||
								(lRef.has_on_negative_side(a) && lRef.has_on_negative_side(b))
						) {
							// no intersection
						} else {
							Pi_2 = intersectBisectorArc(bis,*arc);
						}
					} else {
						Pi_2 = intersectBisectorArc(bis,*arc);
					}
				}

				if(Pi_2 != INFPOINT) {
					piReached = true;

					/* we found two points Pi and Pi_2, one on each chain */
					bool choosePi = false;
					if(bis.isParallel()) {
						Line lRef(sourceNode->point,data.monotonicityLine.direction());
						auto dPi   = normalDistance(lRef,Pi);
						auto dPi_2 = normalDistance(lRef,Pi_2);
						choosePi = (dPi < dPi_2) ? true : false;
					} else {
						/* equality check is difficult for bisector intersections, let us
						 * check first if the next faces, i.e., the respective input edges
						 * are collinear */
						if(!bis.isGhost() && areNextInputEdgesCollinear()) {
							LOG(INFO) << "enter the next edges collinear clause";
							/* in this case we want both arcs in the return set and finish 'here' */
							choosePi  = true;
							piReached = true;
							path = (localOnUpperChain) ? &wf.lowerPath : &wf.upperPath;

							arcs.push_back(path->currentArcIdx);
						} else {
							choosePi = (data.monotoneSmaller(Pi,Pi_2)) ? true : false;
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
			}

			if(!piReached) {
				Point Pr = wf.nodes[wf.getRightmostNodeIdxOfArc(*arc)].point;

				bool classicalSweep = true;
				auto arcOpposite = (arc == arc_l) ? arc_u : arc_l;
				LOG(INFO) << "no intersection found with arc:" << *arc << ", arc on other chain: " << *arcOpposite;

				/*********************************************************/
				/*    check for possible upcoming ghost arc scenario     */
				/*********************************************************/
				if(wf.isArcPerpendicular(*arc_l) || wf.isArcPerpendicular(*arc_u)) {
					if(hasCollinearEdges(*arc,*arcOpposite)) {
						classicalSweep = false;
						LOG(INFO) << "collinear input edges!";
					}
					LOG(INFO) << "possible ghost arc ahead!"; fflush(stdout);
				}

				LOG(INFO) << "comparing points: " << Pi << " and " << Pr;

				if( !classicalSweep || ((data.monotoneSmaller(Pi,Pr)) ||
					(arc->isRay() && !data.rayPointsLeft(arc->ray))) ) {
					LOG(INFO) << "no 2nd intersection but height of Pi reached";
					piReached = true;
					path = (localOnUpperChain) ? &wf.upperPath : &wf.lowerPath;
				}
			}

			if(!piReached) {
				/* check input edge intersection first */
				Edge e = data.getEdge(path->edgeIdx);

				if(localOnUpperChain) {
					pathBackupUpper = wf.upperPath;
				} else {
					pathBackupLower = wf.lowerPath;
				}

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

		if(!arcs.empty() && localOnUpperChain) {
			arcs.insert(arcs.begin(),path->currentArcIdx);
		} else {
			arcs.push_back(path->currentArcIdx);
		}
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

bool Skeleton::hasEquidistantInputEdges(const MonotonePathTraversal& path, const Arc& arc, const Bisector& bis) const {
	std::set<uint> edgeIndicesSet = {arc.leftEdgeIdx,arc.rightEdgeIdx,bis.eIdxA,bis.eIdxB};
	std::vector<uint> edgeIndices(edgeIndicesSet.begin(),edgeIndicesSet.end());
	std::vector<Edge> edges;
	for(auto idx : edgeIndices) {
		edges.push_back(data.getEdge(idx));
	}
	std::vector<std::array<int,2>> idxPairs = {{{0,1}},{{0,2}},{{1,2}}};
	Exact dist = Exact(-1);
	std::vector<Exact> distances;
	for(auto ip : idxPairs) {
		if(!data.isEdgeCollinear(ip[0],ip[1]) && isLinesParallel(edges[ip[0]],edges[ip[1]])) {
			dist = CGAL::squared_distance(edges[ip[0]].supporting_line(),edges[ip[1]].supporting_line());
			if(dist > Exact(0)) {
				distances.push_back(dist);
			}
		}
	}

	if(distances.size() > 0) {
		LOG(INFO) << "number of computed distances: " << distances.size();
		assert(distances.size() > 1);
		return distances[0] == distances[1];
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

bool Skeleton::handleSourceGhostNode(Bisector& bis, std::vector<uint>& arcs, Point& newPoint) {
	if(sourceNode->isGhostNode()) {
		/* we already have a ghost node to handle, i.e., next intersection gives us the 'height' of 'bis'
		 * since the actual height is not known yet  */
		auto arc_u = wf.getArc(wf.upperPath);
		auto arc_l = wf.getArc(wf.lowerPath);

		if(arc_u == nullptr || arc_l == nullptr) {
			return false;
		}

		auto pAA = intersectArcArc(*arc_u, *arc_l);
		if(pAA != INFPOINT) {
			/* now we have to modify the sourcenode 'height' and its incident arcs */
			bis.newSource(pAA);
			/* since sourcenode is a ghost node, its incident edges are collinear*/
			auto arcOfSN = getArc(sourceNode->arcs.front());
			auto bisOfarcOfSN = wf.constructBisector(arcOfSN.leftEdgeIdx,arcOfSN.rightEdgeIdx);

			Point pSN_new = INFPOINT;
			if(bis.supporting_line().is_horizontal() && bisOfarcOfSN.isParallel()) {
				pSN_new = Point(arcOfSN.point(0).x(),bis.point(0).y());
			} else {
				pSN_new = intersectElements(bis.supporting_line(),arcOfSN.supporting_line());
			}
			if(pSN_new != INFPOINT) {
				LOG(WARNING) << "handleGhostVertex: set ghost node to " << pSN_new;
				sourceNode->point = pSN_new;

				/* update arcs to end at the new loci of soucenode */
				for(auto idx : sourceNode->arcs) {
					wf.updateArcNewNode(idx,sourceNodeIdx);
				}

				/* setting the return values */
				arcs.push_back(wf.upperPath.currentArcIdx);
				arcs.push_back(wf.lowerPath.currentArcIdx);
				newPoint = pAA;
				return true;
			} else {
				LOG(WARNING) << "handleGhostVertex: should not happen!"; fflush(stdout);
			}
		} else {
			LOG(WARNING) << "handleGhostVertex: arcs do not intersect!"; fflush(stdout);
		}
	}
	return false;
}

Point Skeleton::handleGhostVertex(const MonotonePathTraversal& path, const Arc& arc, Bisector& bis) {
	if(bis.isParallel() && (wf.isArcPerpendicular(arc))) {
		/* if we have collinear bisectors/arcs then three input edges must be equidistant */
		if(hasEquidistantInputEdges(path,arc,bis)) {
			LOG(WARNING) << "possible ghost arc ahead!";
			/* we add a ghost node that lies between the last added node and the closest endpoint of arc */
			/* we move this point later when we know where it lies */
			Point pA = arc.point(0);
			Point pB = arc.point(1);
			Point pS = sourceNode->point;
			LOG(INFO) << "point A " << pA << " B " << pB << " S " << pS; fflush(stdout);
			Point closestPoint = INFPOINT;
			if(data.monotonicityLine.to_vector().x() > Exact(0)) {
				closestPoint = (CGAL::abs(pS.y()-pA.y()) <= CGAL::abs(pS.y()-pB.y())) ? pA : pB;
			} else {
				closestPoint = (CGAL::abs(pS.x()-pA.x()) <= CGAL::abs(pS.x()-pB.x())) ? pA : pB;
			}
			addGhostNode = true;
			return CGAL::midpoint(pS,closestPoint);
		}
	}

	return INFPOINT;
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
