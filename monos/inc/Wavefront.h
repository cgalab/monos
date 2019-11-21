#ifndef WAVEFRONT_H_
#define WAVEFRONT_H_

#include "cgTypes.h"
#include "Definitions.h"
#include "Data.h"

class Wavefront {

enum class STATE : ul {LOWER=0,UPPER,MERGE};

public:
	STATE state = STATE::LOWER;

	void nextState() {
		if(state == STATE::LOWER) {
			state = STATE::UPPER;
		} else if(state == STATE::UPPER) {
			state = STATE::MERGE;
		}
	}

	Wavefront(Data& dat):data(dat) {}

	bool InitSkeletonQueue(Chain& chain);
	bool SingleDequeue(Chain& chain);
	bool FinishSkeleton(Chain& chain);

	void HandleSingleEdgeEvent(Chain& chain, const Event* event);
	void HandleMultiEdgeEvent(Chain& chain, std::vector<Event*> eventList);
	void HandleMultiEvent(Chain& chain, std::vector<Event*> eventList);

	void InitializeEventsAndPathsPerEdge();
	void InitializeNodes();

	void ChainDecomposition();
	bool ComputeSkeleton(ChainType type);

	Chain& getChain(ChainType type) {return (type == ChainType::UPPER) ? upperChain : lowerChain;}

	Event getEdgeEvent(const ul& aIdx, const ul& bIdx, const ul& cIdx, const ChainRef& it) const;
	void updateNeighborEdgeEvents(const Event& event, const Chain& chain);
	void updateInsertEvent(Event& event);

	void disableEdge(ul edgeIdx) {
		events[edgeIdx].eventPoint = INFPOINT;
		events[edgeIdx].eventTime = 0;
	}

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

	ul getNextArcIdx(const ul& path, bool forward, ul edgeIdx);

	bool isLowerChain(const Chain& chain) const { return &chain == &lowerChain; }

	/* the chain skeleton and the final skeleton is stored in nodes and arcList */
	Nodes				nodes;
	ArcList				arcList;
	/* helping to find the paths, holds for every edge of polygon
	 * the index to the last node on the left/right path */
	PathFinder 			pathFinder;

	/* EVENT QUEUE --------------------------------------------------------------------
	 * Events stored in events, times sorted in eventTimes, with associated edge idx
	 * thus, we can modify the event queue with logarithmic update remove insert times.
	 */
	Events 			events;
	EventTimes 		eventTimes;
	NT				currentTime;
	/* for a polygon edge at position idx in data.polygon we have a respective event
	 * for that edge at position idx as well */

	/* MISC */
	void printChain(const Chain& chain) const;
	void printEvents() const;
//	void printAllArcs() {
//		for(auto a : arcList) {
//			LOG(WARNING) << a;
//		}
//	}

private:
	Segment restrictRay(const Ray& ray);

	Chain  			upperChain, lowerChain;
	Data&    		data;
};

#endif /* WAVEFRONT_H_ */
