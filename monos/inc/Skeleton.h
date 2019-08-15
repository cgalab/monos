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
	using ArcList = std::set<uint>;
public:
	Intersection(Point intersection = INFPOINT):intersection(intersection) {}
	~Intersection() {}

	void addArc(uint arcIdx) {assert(!done); arcs.insert(arcIdx);}
	void clear() {arcs.clear();}

	void add(Point intersection, uint arcIdx) {
		assert(!done);
		setIntersection(intersection);
		addArc(arcIdx);
	}

	void setIntersection(Point P) {intersection = P;}
	Point getIntersection() const {return intersection;}

	void setDone() {done=true;}
	bool isDone() {return done;}

	uint size() const {return arcs.size();}
	bool empty() const {return arcs.empty();}
	ArcList getArcs() const {return arcs;}

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

	/* running the merge by one call */
	void MergeUpperLowerSkeleton();

	void initMerge();
	bool SingleMergeStep();
	void finishMerge();

	void printSkeleton() const;
	void writeOBJ(const Config& cfg) const;

	bool computationFinished = false;

private:
	IntersectionPair findNextIntersectingArc(Bisector& bis);
	bool isIntersectionSimple(const IntersectionPair& pair) const;
	bool isVerticalIntersectionButSimple(const Bisector& bis, const IntersectionPair& pair) const;

	Intersection getIntersectionIfSimple(const Bisector& bis, const IntersectionPair& pair, bool& onUpperChain) const;


	bool removePath(const uint& arcIdx, const uint& edgeIdx);

	uint handleMerge(const Intersection& intersection, const uint& edgeIdxA, const uint& edgeIdxB, const Bisector& bis);
	void updateArcTarget(const uint& arcIdx, const uint& edgeIdx, const int& secondNodeIdx, const Point& edgeEndPoint);

	uint nextUpperChainIndex(const uint& idx) const;
	uint nextLowerChainIndex(const uint& idx) const;

	uint mergeStartNodeIdx() const {return data.e(wf.startLowerEdgeIdx)[0];}
	uint mergeEndNodeIdx() const {return data.e(wf.endLowerEdgeIdx)[1];}
	inline bool isMergeStartEndNodeIdx(const uint& idx) const {return idx == mergeStartNodeIdx() || idx == mergeEndNodeIdx();}

	bool isValidArc(const uint& arcIdx) const {return arcIdx < wf.arcList.size();}
	Arc& getArc(const uint& arcIdx) {assert(isValidArc(arcIdx)); return wf.arcList[arcIdx];}
	Point& getSourceNodePoint() const { return wf.nodes[sourceNodeIdx].point; }
	inline Point intersectArcRay(const Arc& arc, const Ray& ray) const {
		return (arc.type == ArcType::NORMAL) ? intersectElements(ray, arc.edge) : intersectElements(ray, arc.ray);
	}

	bool EndOfOneChains()  const {return EndOfUpperChain() || EndOfLowerChain(); }
	bool EndOfBothChains() const {return EndOfUpperChain() && EndOfLowerChain(); }
	bool EndOfUpperChain() const {return upperChainIndex == wf.startUpperEdgeIdx;}
	bool EndOfLowerChain() const {return lowerChainIndex == wf.endLowerEdgeIdx;  }
	bool EndOfChain(bool upper) { return (upper) ? EndOfUpperChain() : EndOfLowerChain();}

	bool hasCollinearEdges(const Arc& arcA, const Arc& arcB) const;
	void CheckAndResetPath(MonotonePathTraversal* path, const MonotonePathTraversal& pathBackup, const Point& p);

	bool hasPathReachedPoint(const MonotonePathTraversal& path, const Point& P) const;

	bool hasEquidistantInputEdges(const Arc& arc, const Bisector& bis) const;
	bool isNodeIntersectionAndVerticalBisector(const Bisector& bis, const uint nodeIdx) const;

	void reevaluateIntersectionIfMultipleArcs(const Bisector& bis, Intersection& intersection);

	bool areNextInputEdgesCollinear() const;
	bool handleGhostVertex(const MonotonePathTraversal& path, Bisector& bis, Intersection& intersection);
	void handleSourceGhostNode(Bisector& bis, Intersection& intersection);

	void checkNodeIntersection(Intersection& intersection, const Arc* arc);

	void initNextChainAndPath(bool upperChain);

	Data& data;
	Wavefront& 	wf;

	Node* sourceNode = nullptr;
	uint sourceNodeIdx = 0, newNodeIdx = 0;
	uint startIdxMergeNodes = 0;

	/* adding ghost node */
	bool addGhostNode = false;

	uint upperChainIndex = 0, lowerChainIndex = 0;
};

#endif /* SKELETON_H_ */
