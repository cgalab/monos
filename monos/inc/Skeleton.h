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


/* FIRST is UpperChainIntersection / SECOND is LowerChainIntersection*/
using IntersectionPair = std::pair<Point,Point>;

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

	void writeOBJ(const Config& cfg) const;

	bool computationFinished = false;

	void storeChains(Chain upper, Chain lower) {
		upperChain = upper;
		lowerChain = lower;
	}

private:
	/* return std::pair upper/lower intersection Point */
	IntersectionPair findNextIntersectingArc(const Ray& bis);

	bool removePath(const ul& arcIdx, const ul& edgeIdx);

	ul handleMerge(const IntersectionPair& intersectionPair, const Ray& bis);
	void updateArcTarget(const ul& arcIdx, const ul& edgeIdx, const int& secondNodeIdx, const Point& edgeEndPoint);

	inline bool isValidArc(const ul& arcIdx) const {return arcIdx < wf.arcList.size();}

	bool EndOfOneChains()  const {return EndOfUpperChain() || EndOfLowerChain();  }
	bool EndOfBothChains() const {return EndOfUpperChain() && EndOfLowerChain();  }
	bool EndOfUpperChain() const {return upperChainIndex == upperChain.front(); }
	bool EndOfLowerChain() const {return lowerChainIndex == lowerChain.back(); }
	bool EndOfChain(ChainType t) { return (t == ChainType::UPPER) ? EndOfUpperChain() : EndOfLowerChain();}

	Arc& getArc(ul arcIdx) {assert(arcIdx < wf.arcList.size()); return wf.arcList[arcIdx];}

	bool initPathForEdge(ChainType type, const ul& edgeIdx);

	bool isIntersecting(const Ray& ray, const Arc& arc) {
		if(arc.isEdge()) {
			return CGAL::do_intersect(ray,arc.segment);
		}
		return CGAL::do_intersect(ray,arc.ray);
	}



	Data& 		data;
	Wavefront& 	wf;

	Chain  		upperChain, lowerChain;

	/* using as state for the merge */
	Node* sourceNode = nullptr;
	ul sourceNodeIdx = 0, newNodeIdx = 0;

	ul upperChainIndex = 0, lowerChainIndex = 0;
};

#endif /* SKELETON_H_ */
