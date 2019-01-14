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

	void InitializeEventsAndPathsPerEdge();
	void InitializeNodes();

	void ChainDecomposition();
	bool ComputeSkeleton(bool lower);

	Event getEdgeEvent(const uint& aIdx, const uint& bIdx, const uint& cIdx) const;
	void updateNeighborEdgeEvents(const Event& event, const Chain& chain);
	void updateInsertEvent(const Event& event);

	Chain& getUpperChain() { return upperChain; }
	Chain& getLowerChain() { return lowerChain; }
	Line getWeightedOffsetLine(const uint& i) const;

	Ray constructBisector(const uint& aIdx, const uint& bIdx) const;
	void disableEdge(uint edgeIdx) {events[edgeIdx].eventPoint = INFPOINT; }


	/* construct skeletal structure using nodes and arcs */
	uint addArcRay(const uint& nodeAIdx, const uint& edgeLeft, const uint& edgeRight, const Ray& ray);
	uint addArc(const uint& nodeAIdx, const uint& nodeBIdx, const uint& edgeLeft, const uint& edgeRight);
	void addNewNodefromEvent(const Event&, PartialSkeleton& skeleton);

	bool isArcInSkeleton(const uint& arcIdx) const;

	Node& getTerminalNodeForVertex(const uint& vertexIdx)  {return nodes[vertexIdx];}
	void SortArcsOnNodes();

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
