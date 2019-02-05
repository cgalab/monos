#ifndef WAVEFRONT_H_
#define WAVEFRONT_H_

#include "cgTypes.h"
#include "Data.h"

class MonotonePathTraversal {
public:
	MonotonePathTraversal(uint edgeIdx=0, uint currentArcIdx=0, uint oppositeArcIdx=0)
	: edgeIdx(edgeIdx)
	, currentArcIdx(currentArcIdx)
	, oppositeArcIdx(oppositeArcIdx) {}

	bool done() const {return currentArcIdx == oppositeArcIdx;}
	void swap() { std::swap(currentArcIdx, oppositeArcIdx); }

	uint edgeIdx;
	uint currentArcIdx, oppositeArcIdx;

	friend std::ostream& operator<< (std::ostream& os, const MonotonePathTraversal& path);
};

class Wavefront {
public:
	using Events		     = std::vector<Event>;
	using EventTimes 	     = std::set<TimeEdge,TimeEdgeCmp>;

	Wavefront(Data& dat):
		startLowerEdgeIdx(0),endLowerEdgeIdx(0),
		startUpperEdgeIdx(0),endUpperEdgeIdx(0),
		data(dat) {}

	bool InitSkeletonQueue(Chain& chain, PartialSkeleton& skeleton);
	bool SingleDequeue(Chain& chain, PartialSkeleton& skeleton);
	bool FinishSkeleton(Chain& chain, PartialSkeleton& skeleton);

	bool ComputeSingleSkeletonEvent(bool lower);

	void InitializeEventsAndPathsPerEdge();
	void InitializeNodes();

	void ChainDecomposition();
	bool ComputeSkeleton(bool lower);

	Event getEdgeEvent(const uint& aIdx, const uint& bIdx, const uint& cIdx, const ChainRef& it) const;
	void updateNeighborEdgeEvents(const Event& event, const Chain& chain);
	void updateInsertEvent(const Event& event);

	Chain& getUpperChain() { return upperChain; }
	Chain& getLowerChain() { return lowerChain; }

	Ray constructBisector(const uint& aIdx, const uint& bIdx) const;
	void disableEdge(uint edgeIdx) {events[edgeIdx].eventPoint = INFPOINT; }

	/* call simplification from monos class */
	bool InitSkeletonQueue(bool lower) {
		Chain* chain     		   = (lower) ? &getLowerChain()	: &getUpperChain();
		PartialSkeleton* skeleton  = (lower) ? &lowerSkeleton 	: &upperSkeleton;
		return InitSkeletonQueue(*chain,*skeleton);
	}
	bool SingleDequeue(bool lower) {
		Chain* chain   			   = (lower) ? &getLowerChain()	: &getUpperChain();
		PartialSkeleton* skeleton  = (lower) ? &lowerSkeleton 	: &upperSkeleton;
		return SingleDequeue(*chain,*skeleton);
	}
	bool FinishSkeleton(bool lower) {
		Chain* chain     		   = (lower) ? &getLowerChain()	: &getUpperChain();
		PartialSkeleton* skeleton  = (lower) ? &lowerSkeleton 	: &upperSkeleton;
		return FinishSkeleton(*chain,*skeleton);
	}

	/* construct skeletal structure using nodes and arcs */
	uint addArcRay(const uint& nodeAIdx, const uint& edgeLeft, const uint& edgeRight, const Ray& ray);
	uint addArc(const uint& nodeAIdx, const uint& nodeBIdx, const uint& edgeLeft, const uint& edgeRight);
	void addNewNodefromEvent(const Event&, PartialSkeleton& skeleton);
	inline uint addNode(const Point& intersection, const Exact& time) {
		Node node(NodeType::NORMAL,intersection,time);
		nodes.push_back(node);
		return nodes.size() - 1;
	}

	bool isArcInSkeleton(const uint& arcIdx) const;
	inline bool liesOnFace(const Arc& arc, const uint edgeIdx) const {
		return arc.leftEdgeIdx == edgeIdx || arc.rightEdgeIdx == edgeIdx;
	}

	Exact getTime() const {return currentTime;}

	Node& getTerminalNodeForVertex(const uint& vertexIdx)  {return nodes[vertexIdx];}
	void SortArcsOnNodes();

	Arc* getLastArc() {return &arcList[arcList.size()-1];}
	Arc* getArc(const MonotonePathTraversal& path) {return &arcList[path.currentArcIdx];}

	/* -- monotone path traversal -- */
	/* for the merge we have to traverse the faces of a chain-skeleton from 'left to right'
	 * with respect to the monotonicity line. Actually only the right path suffices! */
	bool nextMonotoneArcOfPath(MonotonePathTraversal& path);
	bool isArcLeftOfArc(const Arc& arcA, const Arc& arcB) const;
	const Node& getRightmostNodeOfArc(const Arc& arc) const;
	void initPathForEdge(const bool upper, const uint edgeIdx);
	uint getPossibleRayIdx(const Node& node, uint edgeIdx) const;

	/* the chain skeleton and the final skeleton is stored in nodes and arcList */
	Nodes				nodes;
	ArcList				arcList;
	/* helping to find the paths, holds for every edge of polygon
	 * the index to the last node on the left/right path */
	PathFinder 			pathFinder;
	PartialSkeleton		upperSkeleton, 	lowerSkeleton;

	uint startLowerEdgeIdx, endLowerEdgeIdx;
	uint startUpperEdgeIdx, endUpperEdgeIdx;
	Chain  		upperChain, lowerChain;

	/* for the merge to keep track of the current state */
	MonotonePathTraversal upperPath, lowerPath;

	/* MISC */
	void reset(); /* for GUI version only, to redo the computation without restart */

private:
	/* EVENT QUEUE --------------------------------------------------------------------
	 * Events stored in events, times sorted in eventTimes, with associated edge idx
	 * thus, we can modify the event queue with logarithmic update remove insert times.
	 */
	Events 			events;
	EventTimes 		eventTimes;
	Exact			currentTime;
	/* for a polygon edge at position idx in data.polygon we have a respective event
	 * for that edge at position idx as well */

	Data&     data;
};

#endif /* WAVEFRONT_H_ */
