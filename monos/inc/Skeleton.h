#ifndef SKELETON_H_
#define SKELETON_H_

#include <algorithm>
#include <vector>

/* we store the skeletal structure in two types: nodes and arcs
 * a node holds a vector with the incidences to the incident arcs
 * these arcs are/can be sorted CCW around a node
 * an arc references its 'first' and 'second' node as well as
 * the left and right 'face' (edge, since every edge has only one face) */

#include "tools.h"
#include "Config.h"

#include "cgTypes.h"

#include "Data.h"
#include "Wavefront.h"

class Intersection {
	using ArcList = std::set<ul>;
public:
	Intersection(Point intersection = INFPOINT):intersection(intersection) {}
	~Intersection() {}

//	void addArc(ul arcIdx) {
//	//	assert(!done);
//		arcs.insert(arcIdx);
//	}
//	void clear() {arcs.clear();}
//
//	void add(Point intersection, ul arcIdx) {
//		//assert(!done);
//		setIntersection(intersection);
//		addArc(arcIdx);
//	}
//
//	void remove(const ul arcIdx) {
//		auto it = arcs.find(arcIdx);
//		if(it != arcs.end()) {
//			arcs.erase(it);
//		}
//	}
//
//	void setIntersection(Point P) {intersection = P;}
//	Point getIntersection() const {return intersection;}
//
//	void setDone() {done=true;}
//	bool isDone() {return done;}
//
//	ul size() const {return arcs.size();}
//	bool empty() const {return arcs.empty();}
//	ArcList getArcs() const {return arcs;}
//	ul getFirstArcIdx() const {return (!empty()) ? *arcs.begin() : MAX;}
//	ul getSecondArcIdx() const {return (size()>1) ? *(++arcs.begin()) : MAX;}

	friend std::ostream& operator<< (std::ostream& os, const Intersection& intersection);

private:
	ArcList arcs;
	Point intersection = INFPOINT;
	bool done = false;
};

/* FIRST is UpperChainIntersection / SECOND is LowerChainIntersection*/
using IntersectionPair = std::pair<Intersection,Intersection>;

class Skeleton {
public:
	Skeleton(Data& _data, Wavefront& _wf) :
		data(_data),wf(_wf) {}

	~Skeleton() {}

//	/* running the merge by one call */
//	void MergeUpperLowerSkeleton();
//
//	void initMerge();
//	bool SingleMergeStep();
//	void finishMerge();
//
//	void printSkeleton() const;
//	void writeOBJ(const Config& cfg) const;

	bool computationFinished = false;

private:
//	IntersectionPair findNextIntersectingArc(Bisector& bis);
//
//	bool isIntersectionSimple(IntersectionPair& pair, const Bisector& bis) const;
//	Intersection getIntersectionIfSimple(const Bisector& bis, const IntersectionPair& pair, bool& onUpperChain) const;
//
//	bool removePath(const ul& arcIdx, const ul& edgeIdx);
//
//	ul handleDoubleMerge(IntersectionPair& intersectionPair, const ul& edgeIdxA, const ul& edgeIdxB, const Bisector& bis);
//	ul handleMerge(const Intersection& intersection, const ul& edgeIdxA, const ul& edgeIdxB, const Bisector& bis);
//	void updateArcTarget(const ul& arcIdx, const ul& edgeIdx, const int& secondNodeIdx, const Point& edgeEndPoint);
//
//	ul nextUpperChainIndex(const ul& idx) const;
//	ul nextLowerChainIndex(const ul& idx) const;
//
//	ul mergeStartNodeIdx() const {return data.bbox->monMin.id;}
//	ul mergeEndNodeIdx() const {return data.bbox->monMax.id;}
//	inline bool isMergeStartEndNodeIdx(const ul& idx) const {return idx == mergeStartNodeIdx() || idx == mergeEndNodeIdx();}
//
//	bool isValidArc(const ul& arcIdx) const {return arcIdx < wf.arcList.size();}
//	Arc& getArc(const ul& arcIdx) {assert(isValidArc(arcIdx)); return wf.arcList[arcIdx];}
//	Point& getSourceNodePoint() const { return wf.nodes[sourceNodeIdx].point; }
//	inline Point intersectArcRay(const Arc& arc, const Ray& ray) const {
//		return (arc.type == ArcType::NORMAL) ? intersectElements(ray, arc.edge) : intersectElements(ray, arc.ray);
//	}
//
//	bool EndOfOneChains()  const {return EndOfUpperChain() || EndOfLowerChain(); }
//	bool EndOfBothChains() const {return EndOfUpperChain() && EndOfLowerChain(); }
//	bool EndOfUpperChain() const {return upperChainIndex == data.e(wf.upperChain.back()).id;}
//	bool EndOfLowerChain() const {return lowerChainIndex == data.e(wf.lowerChain.back()).id;  }
//	bool EndOfChain(bool upper) { return (upper) ? EndOfUpperChain() : EndOfLowerChain();}
//
//	bool hasCollinearEdges(const Arc& arcA, const Arc& arcB, bool avoidChainEdges=false) const;
//	void CheckAndResetPath(MonotonePathTraversal* path, const MonotonePathTraversal& pathBackup, const Point& p);
//
//	bool hasPathReachedPoint(const MonotonePathTraversal& path, const Point& P) const;
//
//	bool hasEquidistantInputEdges(const Arc& arc, const Bisector& bis) const;
//	bool isNodeIntersectionAndVerticalBisector(const Bisector& bis, const ul nodeIdx) const;
//
//	void reevaluateIntersectionIfMultipleArcs(const Bisector& bis, Intersection& intersection);
//	void multiEventCheck(const Bisector& bis, IntersectionPair& pair);
//	void checkAndHandlePossibleSourceGhostNode(IntersectionPair& pair, Bisector& bis);
//
//	bool hasArcCurrentChainIndices(const Arc& arc) const {
//		return arc.leftEdgeIdx == lowerChainIndex || arc.rightEdgeIdx == lowerChainIndex || arc.leftEdgeIdx == upperChainIndex || arc.rightEdgeIdx == upperChainIndex;
//	}
//	bool areNextInputEdgesCollinear() const;
//	bool handleGhostVertex(const MonotonePathTraversal& path, Bisector& bis, Intersection& intersection);
//	void handleSourceGhostNode(Bisector& bis, IntersectionPair& pair);
//
//	bool secondIntersectionDoneAndWeCatchedUp(Bisector& bis, Intersection& upper, Intersection& lower, const Arc& arc_u, const Arc& arc_l, const bool localOnUpperChain) const;
//	void checkNodeIntersection(Intersection& intersection, const Arc* arc);
//
//	void removeRaysFromIntersection(Intersection& intersection);
//
//	ul getNextEdgeIdxFromIntersection(const Intersection& intersection, bool onUpperChain);
//
//	ul getAVerticalArc(const Intersection& intersection) const;
//	bool intersectionHasVerticalArc(const IntersectionPair& pair) const;
//	bool intersectionHasVerticalCoallignedArc(const IntersectionPair& pair) const;
//
//	void nodeZeroEdgeRemoval(Node& node);
//
//	void initNextChainAndPath(bool upperChain);

	Data& 		data;
	Wavefront& 	wf;

	Node* sourceNode = nullptr;
	ul sourceNodeIdx = 0, newNodeIdx = 0;
	ul startIdxMergeNodes = 0;

	/* adding ghost node */
	bool addGhostNode = false;

	ul upperChainIndex = 0, lowerChainIndex = 0;
};

#endif /* SKELETON_H_ */
