/* monos is written in C++.  It computes the weighted straight skeleton
 * of a monotone polygon in asymptotic n log n time and linear space.
 *
 * Copyright 2018, 2019 Günther Eder - geder@cs.sbg.ac.at
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WAVEFRONT_H_
#define WAVEFRONT_H_

#include "cgTypes.h"
#include "Definitions.h"
#include "Data.h"

#include "EventQueue.h"

class Wavefront {

enum class STATE : ul {LOWER=0,UPPER,MERGE};

public:
	STATE state = STATE::LOWER;

	void nextState() {
		if(state == STATE::LOWER) {
			delete eventTimes;
			state = STATE::UPPER;
		} else if(state == STATE::UPPER) {
			state = STATE::MERGE;
		}
	}

	Wavefront(Data& dat):data(dat) {}
	~Wavefront() {delete eventTimes;}
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

	inline Line getNormalBisector(const ul& aIdx, const ul& bIdx, const Line& l) const {
		return l.perpendicular(nodes[pathFinder[aIdx].b].point);
	}

	Event getEdgeEvent(const ul& aIdx, const ul& bIdx, const ul& cIdx, const ChainRef& it) const;
	void updateNeighborEdgeEvents(const Event& event, const Chain& chain);
	void updateInsertEvent(Event& event);

	inline void disableEdge(ul edgeIdx) {
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

	inline NT getTime() const {return currentTime;}

	inline Node& getTerminalNodeForVertex(const ul& vertexIdx)  {return nodes[vertexIdx];}

	inline Node* getNode(const ul& idx) {return &nodes[idx];}
	inline Arc* getArc(const ul& idx) {assert(idx < arcList.size()); return &arcList[idx];}

	ul getNextArcIdx(const ul& path, bool forward, ul edgeIdx);

	inline bool isLowerChain(const Chain& chain) const { return &chain == &lowerChain; }

	/* the chain skeleton and the final skeleton is stored in nodes and arcList */
	Nodes				nodes;
	ArcList				arcList;
	/* helping to find the paths, holds for every edge of polygon
	 * the index to the last node on the left/right path */
	PathFinder 			pathFinder;

	/* EVENT QUEUE --------------------------------------------------------------------
	 * Events stored in events, priority queue is weasel's heap -> 'heap.h'
	 * we place Event* in the Queue Items and sort by eventTime
	 */
	Events 			events;
	EventQueue 		*eventTimes = nullptr;
	NT				currentTime = 0;

	template<class T, class U>
	inline bool isCollinear(const T& a, const U& b) const {
		return CGAL::collinear(a.point(0),a.point(1),b.point(0)+b.to_vector());
	}

	/* MISC */
	void printChain(const Chain& chain) const;
	void printEvents() const;

private:
	Segment restrictRay(const Ray& ray);

	Chain  			upperChain, lowerChain;
	Data&    		data;
};

#endif /* WAVEFRONT_H_ */
