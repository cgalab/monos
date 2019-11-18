#include "BasicInput.h"

void
BasicInput::add_graph(const GMLGraph& graph) {
  assert(vertices_.size() == 0);
  assert(edges_.size() == 0);

  typedef GMLGraph::vertex_descriptor VertexType;
  typedef GMLGraph::edge_descriptor EdgeType;

  auto index_map = boost::get(boost::vertex_index, graph);

  for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
    const VertexType v = *vp.first;

    CORE::BigInt px = (CORE::BigInt)(std::stod(graph[v].x) * INTOFFSET);
    CORE::BigInt py = (CORE::BigInt)(std::stod(graph[v].y) * INTOFFSET);

    Point_2 p(px,py);

    unsigned degree = boost::degree(v, graph);
    add_vertex(Vertex(p, degree, vertices_.size()));
    assert(index_map[v] == vertices_.size()-1);
    if (degree == 1) num_of_deg1_vertices++;
  }
  for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
    const EdgeType e = *ep.first;
    const NT weight((graph[e].weight == "") ? CORE_ONE : graph[e].weight);
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
    assert(e.u < vertices_.size());
    assert(e.v < vertices_.size());
    d[e.u]++;
    d[e.v]++;
    auto findres = edge_map.find(VertexIdxPair(e.u,e.v));
    assert(findres != edge_map.end());
    assert(findres->second == i);
  }
  for (size_t i=0; i<vertices_.size(); ++i) {
    const auto & v = vertices_[i];
    assert(d[i] == v.degree);
    if (d[i] == 1) deg1++;
  }
  assert(deg1 == num_of_deg1_vertices);
  delete[] d;
}

void BasicInput::add_list(const PointList& points, const SimpleEdgeList& edges) {
	for(auto point : points) {
		add_vertex(Vertex(point,2,vertices_.size()));
	}
	for(auto edge : edges) {
		add_edge(edge[0],edge[1],CORE_ONE);
	}
}
