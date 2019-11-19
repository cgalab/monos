#pragma once

#include "cgTypes.h"

#include "BGLGraph.h"
#include "tools.h"

#include <map>
#include <set>
#include <utility>


class BasicInput {

private:
	unsigned num_of_deg1_vertices = 0;
	VertexList vertices_;
	EdgeList edges_;
	std::map<VertexIdxPair, unsigned> edge_map;

	/** Add an input vertex to the vertexlist */
	inline void add_vertex(Vertex&& p) {
		vertices_.emplace_back(std::forward<Vertex>(p));
	}
	/** Add an input edge between vertices to the edgelist */
	inline void add_edge(unsigned u, unsigned v) {
		assert(u < vertices_.size());
		assert(v < vertices_.size());
		assert(u!=v);
//		LOG(INFO) << "add e " << u << "->" << v;
		edges_.emplace_back(Edge(u,v,edges_.size(), Segment(vertices_[u].p, vertices_[v].p)));

		sort_tuple(u,v);
		auto res = edge_map.emplace(std::pair<VertexIdxPair,unsigned>(VertexIdxPair(u,v), edges().size()-1));
		assert(res.second);
	}

	void assert_valid() const;
public:
	const VertexList& vertices() const { return vertices_; };
	const EdgeList& edges() const { return edges_; };
	void add_graph(const BGLGraph& graph);
	unsigned get_num_of_deg1_vertices() const {
		return num_of_deg1_vertices;
	}
	unsigned get_total_degree() const {
		return edges().size() * 2;
	}
	unsigned get_num_extra_beveling_vertices() const {
		/* Includes the extra one vertex we'll need at a minimum for degree-1 vertices. */
		return num_of_deg1_vertices;
	}
	bool has_edge(unsigned u, unsigned v) const {
		sort_tuple(u,v);
		auto findres = edge_map.find(VertexIdxPair(u, v));
		return findres != edge_map.end();
	}
	const Edge& get_edge(unsigned idx) const {
		assert(idx < edges_.size());
		return edges_[idx];
	}
	const Edge& get_edge(unsigned u, unsigned v) const {
		sort_tuple(u,v);
		assert(has_edge(u,v));
		auto findres = edge_map.find(VertexIdxPair(u, v));
		assert(findres != edge_map.end());
		return edges_[findres->second];
	}

//	Segment get_segment(const Edge& e) const {
//		return Segment(vertices_[e.u].p, vertices_[e.v].p);
//	}
	const Segment& get_segment(const Edge& e) const {
		return e.segment;
	}
};
