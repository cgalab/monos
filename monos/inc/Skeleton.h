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

class Skeleton {
public:
//	Skeleton(const Data& _data, Wavefront& _wf) :
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

private:
	void findNextIntersectingArc(const Ray& bis, std::vector<uint>& arcs, bool& upperChain, Point& newPoint);
	bool nextArcOnPath(const uint& arcIdx, const uint& edgeIdx, uint& nextArcIdx) const;

	uint handleMerge(const std::vector<uint>& arcIndices, const uint& edgeIdxA, const uint& edgeIdxB, const Point& p, const Ray& bis);
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

//	const Data& data;
	Data& data;
	Wavefront& 	wf;

	Node* sourceNode = nullptr;
	uint sourceNodeIdx = 0, newNodeIdx = 0;
	uint startIdxMergeNodes = 0;

	uint upperChainIndex = 0, lowerChainIndex = 0;
};

#endif /* SKELETON_H_ */
