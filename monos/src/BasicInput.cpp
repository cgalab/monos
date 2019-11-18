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
	for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
		const EdgeType e = *ep.first;
		const NT weight((graph[e].weight == "") ? NT(1) : graph[e].weight);
		add_edge(source(e, graph), target(e, graph), weight);
	}

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
		assert(findres != edge_map.end());
		assert(findres->second == i);
	}
	delete[] d;
}
