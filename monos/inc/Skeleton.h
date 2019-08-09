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

	void addArc(uint arcIdx, bool upperChain) {
		if(upperChain) {
			addUpperArc(arcIdx);
		} else {
			addLowerArc(arcIdx);
		}
	}
	void addUpperArc(uint arcIdx) {upperArcs.insert(arcIdx);}
	void addLowerArc(uint arcIdx) {lowerArcs.insert(arcIdx);}

	void clearAll() {clearUpperArcs(); clearLowerArcs();}
	void clear(bool upperChain) {
		if(upperChain) {
			clearUpperArcs();
		} else {
			clearLowerArcs();
		}
	}
	void clearUpperArcs() {upperArcs.clear();}
	void clearLowerArcs() {lowerArcs.clear();}

	void add(Point intersection, uint arcIdx, bool upperChain) {
		setIntersection(intersection,upperChain);
		addArc(arcIdx,upperChain);
	}

	void setIntersection(bool upperChain) {intersection = (upperChain) ? firstUpperIntersection : firstLowerIntersection;}
	void setIntersection(Point newIntersection) {intersection = newIntersection;}
	void setIntersection(Point newIntersection, bool upperChain) {
		if(upperChain) {
			setUpperIntersection(newIntersection);
		} else {
			setLowerIntersection(newIntersection);
		}
	}
	void setUpperIntersection(Point newIntersection) {firstUpperIntersection = newIntersection;}
	void setLowerIntersection(Point newIntersection) {firstLowerIntersection = newIntersection;}

	Point point(bool upperChain) const { return (upperChain) ? upperPoint() : lowerPoint();}
	Point point() const {
		for(auto p : {firstLowerIntersection,firstUpperIntersection,intersection}) {
			if(p != INFPOINT) {
				return p;
			}
		}
		return INFPOINT;
	}

	Point lowerPoint() const {return firstLowerIntersection;}
	Point upperPoint() const {return firstUpperIntersection;}
	uint size() const {return lowerArcs.size() + upperArcs.size();}

	bool empty() const {return upperArcs.empty() && lowerArcs.empty();}
	bool isValid() const {
		return !empty() && (firstLowerIntersection != INFPOINT || firstUpperIntersection != INFPOINT);
	}
	bool isBothIntersectionsValid() const {
		return lowerPoint() != INFPOINT && upperPoint() != INFPOINT && !upperArcs.empty() && !lowerArcs.empty();
	}
	bool isEqualIntersectionPoints() const { return firstLowerIntersection == firstUpperIntersection;}


	bool onlyUpperChain() const {return !empty() && lowerArcs.empty();}
	bool onlyLowerChain() const {return !empty() && upperArcs.empty();}

	ArcList getUpperArcs() const {return upperArcs;}
	ArcList getLowerArcs() const {return lowerArcs;}
	ArcList getAllArcs() const {
		ArcList allArcs(upperArcs);
		std::copy (lowerArcs.begin(),lowerArcs.end(),std::inserter(allArcs,allArcs.end()));
		return allArcs;
//		return ArcList(std::inserter(upperArcs,upperArcs.end()),std::inserter(lowerArcs,lowerArcs.end()));
	}
	Point getUpperIntersection() const {return firstUpperIntersection;}
	Point getLowerIntersection() const {return firstLowerIntersection;}

	friend std::ostream& operator<< (std::ostream& os, const Intersection& intersection);

private:
	ArcList upperArcs;
	ArcList lowerArcs;
	Point intersection = INFPOINT;
	Point firstUpperIntersection = INFPOINT;
	Point firstLowerIntersection = INFPOINT;
};

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
	Intersection findNextIntersectingArc(Bisector& bis);
	void simplifyIntersection(const Bisector& bis, Intersection& intersection);
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

	bool hasEquidistantInputEdges(const MonotonePathTraversal& path, const Arc& arc, const Bisector& bis) const;
	bool areNextInputEdgesCollinear() const;
	bool handleGhostVertex(const MonotonePathTraversal& path, Bisector& bis, Intersection& intersection);
	void handleSourceGhostNode(Bisector& bis, Intersection& intersection);

	void checkNodeIntersection(Intersection& intersection, const Arc* arc, bool localOnUpperChain);
	bool monotoneSmallerPointOnBisector(const Line& bisLine, const Intersection& intersection, const bool localOnUpperChain) const;

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
