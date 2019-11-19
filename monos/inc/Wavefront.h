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
//	bool isAnIndex(const ul idx) const { return idx == currentArcIdx || idx == finalArcIdx; }
//	void swap() { std::swap(currentArcIdx, finalArcIdx); }

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

	Wavefront(Data& dat):
//		startLowerEdgeIdx(0),endLowerEdgeIdx(0),
//		startUpperEdgeIdx(0),endUpperEdgeIdx(0),
		data(dat) {}

	bool InitSkeletonQueue(Chain& chain);
	bool SingleDequeue(Chain& chain);
	bool FinishSkeleton(Chain& chain);

//	bool ComputeSingleSkeletonEvent(bool lower);
	void HandleSingleEdgeEvent(Chain& chain, const Event* event);
	void HandleMultiEdgeEvent(Chain& chain, std::vector<const Event*> eventList);
	void HandleMultiEvent(Chain& chain, std::vector<const Event*> eventList);

	void InitializeEventsAndPathsPerEdge();
	void InitializeNodes();

	void ChainDecomposition();
	bool ComputeSkeleton(ChainType type);

	Chain& getChain(ChainType type) {return (type == ChainType::UPPER) ? upperChain : lowerChain;}
//	PartialSkeleton& getSkeleton(ChainType type) {return (type == ChainType::UPPER) ? upperSkeleton : lowerSkeleton;}

	Event getEdgeEvent(const ul& aIdx, const ul& bIdx, const ul& cIdx, const ChainRef& it) const;
	void updateNeighborEdgeEvents(const Event& event, const Chain& chain);
	void updateInsertEvent(const Event& event);
//
//	Chain& getUpperChain() { return upperChain; }
//	Chain& getLowerChain() { return lowerChain; }
//
//	Bisector constructBisector(const ul& aIdx, const ul& bIdx) const;
//	Bisector getBisectorWRTMonotonicityLine(const Bisector& bisector) const;
//	Point intersectBisectorArc(const Bisector& bis, const Arc& arc);
	void disableEdge(ul edgeIdx) {events[edgeIdx].eventPoint = INFPOINT; }
//
//	/* call simplification from monos class */
//	bool InitSkeletonQueue(bool lower) {
//		Chain* chain     		   = (lower) ? &getLowerChain()	: &getUpperChain();
//		PartialSkeleton* skeleton  = (lower) ? &lowerSkeleton 	: &upperSkeleton;
//		return InitSkeletonQueue(*chain,*skeleton);
//	}
//	bool SingleDequeue(bool lower) {
//		Chain* chain   			   = (lower) ? &getLowerChain()	: &getUpperChain();
//		PartialSkeleton* skeleton  = (lower) ? &lowerSkeleton 	: &upperSkeleton;
//		return SingleDequeue(*chain,*skeleton);
//	}
//	bool FinishSkeleton(bool lower) {
//		Chain* chain     		   = (lower) ? &getLowerChain()	: &getUpperChain();
//		PartialSkeleton* skeleton  = (lower) ? &lowerSkeleton 	: &upperSkeleton;
//		return FinishSkeleton(*chain,*skeleton);
//	}

	/* construct skeletal structure using nodes and arcs */
	ul addArcRay(const ul& nodeAIdx, const ul& edgeLeft, const ul& edgeRight, const Ray& ray);
	ul addArc(const ul& nodeAIdx, const ul& nodeBIdx, const ul& edgeLeft, const ul& edgeRight);
	void addNewNodefromEvent(const Event&);

	inline ul addNode(const Point& intersection, const NT& time, NodeType type = NodeType::NORMAL) {
		nodes.emplace_back(Node(type,intersection,time, nodes.size()));
		return nodes.size() - 1;
	}

//	bool hasArcParallelEdges(const Arc& arc) const {
//		return CGAL::parallel(data.get_segment(arc.leftEdgeIdx).supporting_line(),data.get_segment(arc.rightEdgeIdx).supporting_line());
//	}
//
//	bool isArcPerpendicular(const Arc& arc) const;
//	bool isArcInSkeleton(const ul& arcIdx) const;
	inline bool liesOnFace(const Arc& arc, const ul& edgeIdx) const {
		return arc.leftEdgeIdx == edgeIdx || arc.rightEdgeIdx == edgeIdx;
	}

	NT getTime() const {return currentTime;}
//
	Node& getTerminalNodeForVertex(const ul& vertexIdx)  {return nodes[vertexIdx];}
//	void SortArcsOnNodes();
//	Arc* getLastArc() {return &arcList[arcList.size()-1];}
//
	Node* getNode(const ul& idx) {return &nodes[idx];}
	Arc* getArc(const ul& idx) {assert(idx < arcList.size()); return &arcList[idx];}
//	Arc* getArc(const MonotonePathTraversal& path) {
//		if(path.currentArcIdx == MAX) {
//			return nullptr;
//		} else {
//			return &arcList[path.currentArcIdx];
//		}
//	}
	ul getNextArcIdx(const MonotonePathTraversal& path, const Arc& arc) const;
//
//	/* -- monotone path traversal -- */
//	/* for the merge we have to traverse the faces of a chain-skeleton from 'left to right'
//	 * with respect to the monotonicity line. Actually only the right path suffices! */
//	bool nextMonotoneArcOfPath(MonotonePathTraversal& path);
	bool isArcLeftOfArc(const Line& line, const Arc& arcA, const Arc& arcB) const;
//	bool isArcLeftOfArc(const Arc* arcA, const Arc* arcB) const;
//	bool isArcLeftOfArc(const Arc& arcA, const Arc& arcB) const;
//	bool isArcLeftOfPoint(const Arc& arc, const Point& point) const;
	ul getLeftmostNodeIdxOfArc(const Arc& arc) const;
	ul getRightmostNodeIdxOfArc(const Arc& arc) const;

	void initPathForEdge(ChainType type, const ul& edgeIdx);

//	ul getPossibleRayIdx(const Node& node, ul edgeIdx) const;
//	ul getCommonNodeIdx(const ul& arcIdxA, const ul& arcIdxB);
//
//	void updateArcNewNode(const ul idx, const ul nodeIdx);
//
//
	bool isLowerChain(const Chain& chain) const { return &chain == &lowerChain; }
//	bool isEdgeOnLowerChain(const ul edgeIdx) const;
//
//	bool hasParallelBisector(const Event& event) const;

	/* the chain skeleton and the final skeleton is stored in nodes and arcList */
	Nodes				nodes;
	ArcList				arcList;
	/* helping to find the paths, holds for every edge of polygon
	 * the index to the last node on the left/right path */
	PathFinder 			pathFinder;
//	PartialSkeleton		upperSkeleton, 	lowerSkeleton;


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

private:
	Chain  			upperChain, lowerChain;
	Data&    		data;
};

#endif /* WAVEFRONT_H_ */
