#ifndef WAVEFRONT_H_
#define WAVEFRONT_H_

#include "cgTypes.h"
#include "Data.h"

class Wavefront {
public:
	using Events		     = std::vector<Event>;
	using EventTimes 	     = std::set<TimeEdge,TimeEdgeCmp>;

	Wavefront(const Data& dat):
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
	Line getWeightedOffsetLine(const uint& i) const;

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

	Exact getTime() const {return currentTime;}

	Node& getTerminalNodeForVertex(const uint& vertexIdx)  {return nodes[vertexIdx];}
	void SortArcsOnNodes();

	Arc* getLastArc() {return &arcList[arcList.size()-1];}

	Nodes				nodes;
	ArcList				arcList;
	/* helping to find the paths, holds for every edge of polygon
	 * the index to the last node on the left/right path */
	PathFinder 			pathFinder;
	PartialSkeleton		upperSkeleton, 	lowerSkeleton;

	uint startLowerEdgeIdx, endLowerEdgeIdx;
	uint startUpperEdgeIdx, endUpperEdgeIdx;
	Chain  		upperChain, lowerChain;

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

	const Data&     data;
};

#endif /* WAVEFRONT_H_ */
