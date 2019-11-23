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

	/** Add an input vertex to the vertexlist */
	inline void add_vertex(Vertex&& p) {
		vertices_.emplace_back(std::forward<Vertex>(p));
	}
	/** Add an input edge between vertices to the edgelist */
	inline void add_edge(unsigned u, unsigned v) {
		assert(u < vertices_.size());
		assert(v < vertices_.size());
		assert(u!=v);
		edges_.emplace_back(Edge(u,v,edges_.size(), Segment(vertices_[u].p, vertices_[v].p)));

		sort_tuple(u,v);
	}

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
	const Edge& get_edge(unsigned idx) const {
		assert(idx < edges_.size());
		return edges_[idx];
	}
	const Segment& get_segment(const Edge& e) const {
		return e.segment;
	}
};
