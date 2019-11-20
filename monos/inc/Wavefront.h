#ifndef WAVEFRONT_H_
#define WAVEFRONT_H_

#include "cgTypes.h"
#include "Definitions.h"
#include "Data.h"


class MonotonePathTraversal {
public:
	MonotonePathTraversal(ul edgeIdx=0, ul currentArcIdx=0, ul oppositeArcIdx=0, ChainType type = ChainType::UPPER)
	: edgeIdx(edgeIdx)
	, currentArcIdx(currentArcIdx)
	, finalArcIdx(oppositeArcIdx)
	, type(type)
	{}

	bool done() const {return currentArcIdx == finalArcIdx;}
	bool isUpperChain() const { return ChainType::UPPER == type; }

	void set(const MonotonePathTraversal& reset) {
		edgeIdx 		    = reset.edgeIdx;
		currentArcIdx 	    = reset.currentArcIdx;
		finalArcIdx 	    = reset.finalArcIdx;
		type 		   	 	= reset.type;
	}

	ul edgeIdx;
	ul currentArcIdx, finalArcIdx;
	ChainType type;


	bool operator==(const MonotonePathTraversal& rhs) const {
		return this->currentArcIdx == rhs.currentArcIdx && this->edgeIdx == rhs.edgeIdx && this->finalArcIdx == rhs.finalArcIdx && this->type == rhs.type;
	}
	bool operator!=(const MonotonePathTraversal& rhs) const {
		return !(*this == rhs);
	}

	friend std::ostream& operator<< (std::ostream& os, const MonotonePathTraversal& path);
};

class Wavefront {
public:

	Wavefront(Data& dat):data(dat) {}

	bool InitSkeletonQueue(Chain& chain);
	bool SingleDequeue(Chain& chain);
	bool FinishSkeleton(Chain& chain);

	void HandleSingleEdgeEvent(Chain& chain, const Event* event);
	void HandleMultiEdgeEvent(Chain& chain, std::vector<const Event*> eventList);
	void HandleMultiEvent(Chain& chain, std::vector<const Event*> eventList);

	void InitializeEventsAndPathsPerEdge();
	void InitializeNodes();

	void ChainDecomposition();
	bool ComputeSkeleton(ChainType type);

	Chain& getChain(ChainType type) {return (type == ChainType::UPPER) ? upperChain : lowerChain;}

	Event getEdgeEvent(const ul& aIdx, const ul& bIdx, const ul& cIdx, const ChainRef& it) const;
	void updateNeighborEdgeEvents(const Event& event, const Chain& chain);
	void updateInsertEvent(Event& event);

	void disableEdge(ul edgeIdx) {events[edgeIdx].eventPoint = INFPOINT; }

	/* construct skeletal structure using nodes and arcs */
	ul addArcRay(const ul& nodeAIdx, const ul& edgeLeft, const ul& edgeRight, const Ray& ray);
	ul addArc(const ul& nodeAIdx, const ul& nodeBIdx, const ul& edgeLeft, const ul& edgeRight);
	void addNewNodefromEvent(const Event&);

	inline ul addNode(const Point& intersection, const NT& time, NodeType type = NodeType::NORMAL) {
		nodes.emplace_back(Node(type,intersection,time, nodes.size()));
		return nodes.size() - 1;
	}

	inline bool liesOnFace(const Arc& arc, const ul& edgeIdx) const {
		return arc.leftEdgeIdx == edgeIdx || arc.rightEdgeIdx == edgeIdx;
	}

	NT getTime() const {return currentTime;}

	Node& getTerminalNodeForVertex(const ul& vertexIdx)  {return nodes[vertexIdx];}

	Node* getNode(const ul& idx) {return &nodes[idx];}
	Arc* getArc(const ul& idx) {assert(idx < arcList.size()); return &arcList[idx];}

	ul getNextArcIdx(const MonotonePathTraversal& path, const Arc& arc) const;

	bool isArcLeftOfArc(const Line& line, const Arc& arcA, const Arc& arcB) const;

	ul getLeftmostNodeIdxOfArc(const Arc& arc) const;
	ul getRightmostNodeIdxOfArc(const Arc& arc) const;

	bool isLowerChain(const Chain& chain) const { return &chain == &lowerChain; }

	/* the chain skeleton and the final skeleton is stored in nodes and arcList */
	Nodes				nodes;
	ArcList				arcList;
	/* helping to find the paths, holds for every edge of polygon
	 * the index to the last node on the left/right path */
	PathFinder 			pathFinder;


	/* for the merge to keep track of the current state */
	MonotonePathTraversal upperPath, lowerPath;

	/* MISC */
	void printChain(const Chain& chain) const;
	void printEvents() const;

	/* EVENT QUEUE --------------------------------------------------------------------
	 * Events stored in events, times sorted in eventTimes, with associated edge idx
	 * thus, we can modify the event queue with logarithmic update remove insert times.
	 */
	Events 			events;
	EventTimes 		eventTimes;
	NT				currentTime;
	/* for a polygon edge at position idx in data.polygon we have a respective event
	 * for that edge at position idx as well */

	void printAllArcs() {
		for(auto a : arcList) {
			LOG(WARNING) << a;
		}
	}

private:
	Chain  			upperChain, lowerChain;
	Data&    		data;
};

#endif /* WAVEFRONT_H_ */
