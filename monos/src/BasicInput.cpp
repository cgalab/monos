#include "cgTypes.h"
#include "BasicInput.h"

void
BasicInput::add_graph(const BGLGraph& graph) {
	assert(vertices_.size() == 0);
	assert(edges_.size() == 0);

	typedef BGLGraph::vertex_descriptor VertexType;
	typedef BGLGraph::edge_descriptor EdgeType;

	auto index_map = boost::get(boost::vertex_index, graph);

	for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		const VertexType v = *vp.first;
		Point p(graph[v].x, graph[v].y);
		unsigned degree = boost::degree(v, graph);
		add_vertex(Vertex(p, vertices_.size()));
		assert(index_map[v] == vertices_.size()-1);
		if (degree == 1) num_of_deg1_vertices++;
	}
	std::vector<sl> map(vertices_.size(), NIL);
	for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
		const EdgeType e = *ep.first;
		if(map[source(e, graph)] == NIL) {
			map[source(e, graph)] = target(e, graph);
		} else if(map[target(e, graph)] == NIL) {
			map[target(e, graph)] = source(e, graph);
		} else {
			LOG(WARNING) << "An additional edge? " << source(e, graph) << " -> " << target(e, graph);
		}
	}

	unsigned idx = 0;
	do {
		add_edge(idx,map[idx]);
		idx = map[idx];
	} while(idx != 0);

	assert_valid();
}

void
BasicInput::assert_valid() const {
	unsigned* d = new unsigned[vertices_.size()]();
	unsigned deg1 = 0;

	assert(edge_map.size() == edges_.size());
	for (size_t i=0; i<edges_.size(); ++i) {
		const auto & e = edges_[i];
		LOG(INFO) << i << ": " << e.u << " " << e.v;
		assert(e.u < vertices_.size());
		assert(e.v < vertices_.size());
		d[e.u]++;
		d[e.v]++;
		auto findres = edge_map.find(VertexIdxPair(e.u,e.v));
		if(findres == edge_map.end()) {
			findres = edge_map.find(VertexIdxPair(e.v,e.u));
		}
		assert(findres != edge_map.end());
		assert(findres->second == i);
	}
	delete[] d;
}
