#pragma once

#include <map>
#include <set>
#include <utility>
#include "GMLGraph.h"

#define INTOFFSET 1000000000000000

#ifndef CORE_ONE
#define CORE_ONE NT(1.0)
#endif

class BasicVertex {
  public:
    const Point_2 p;
    const unsigned degree;
    const unsigned id;
    const unsigned reflex_beveling_add;

    BasicVertex(const Point_2& p, unsigned degree, unsigned id)
      : p(p)
      , degree(degree)
      , id(id)
      , reflex_beveling_add(0) // number of kinetic vertices to create additionally at reflex vertices.
                               // At degree one vertices, we always add at least one.
      {}
};

class BasicEdge {
  public:
    unsigned u, v;
    NT weight;

    BasicEdge(unsigned u, unsigned v, const NT &weight=1.0)
      : u(u)
      , v(v)
      , weight(weight)
      {
        assert(u < v);
      }
};

class BasicInput {
  using Vertex = BasicVertex;
  using Edge = BasicEdge;

  using VertexList = std::vector<Vertex>;
  using EdgeList = std::vector<Edge>;
  using VertexIdxPair = std::pair<unsigned,unsigned>;

  using PointList = std::vector<Point_2>;
  using SimpleEdgeList = std::vector<std::array<uint,2> >;

  private:
    unsigned num_of_deg1_vertices = 0;
    VertexList vertices_;
    EdgeList edges_;
    std::map<VertexIdxPair, unsigned> edge_map;
    /* keep a list of instances of our number type for the different weights. */
    std::set<NT> weight_set;

    void assert_valid() const;

  public:
    /** Add an input vertex to the vertexlist */
    inline void add_vertex(Vertex&& p) {
      vertices_.emplace_back(std::forward<Vertex>(p));
    }
    /** Add an input edge between vertices to the edgelist */
    inline void add_edge(unsigned u, unsigned v, const NT& weight=1.0) {
      sort_tuple(u,v);
      assert(u < vertices_.size());
      assert(v < vertices_.size());
      assert(u!=v);

      auto wsinsert_res = weight_set.insert(weight);
      edges_.emplace_back(Edge(u,v, *wsinsert_res.first));
      auto res = edge_map.emplace(std::pair<VertexIdxPair,unsigned>(VertexIdxPair(u,v), edges().size()-1));
      assert(res.second);
    }

    const VertexList& vertices() const { return vertices_; };
    const EdgeList& edges() const { return edges_; };
    void add_graph(const GMLGraph& graph);
    void add_list(const PointList& points, const SimpleEdgeList& edges);
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
    const Edge& get_edge(unsigned u, unsigned v) const {
      sort_tuple(u,v);
      assert(has_edge(u,v));
      auto findres = edge_map.find(VertexIdxPair(u, v));
      assert(findres != edge_map.end());
      return edges_[findres->second];
    }

    void update_edge(unsigned idx, unsigned u, unsigned v) {
    	auto edge = &edges_[idx];
    	sort_tuple(u,v);
    	edge->u = u;
    	edge->v = v;
    }

    void set_weight(unsigned idx, NT weight) {
    	auto edge = &edges_[idx];
    	edge->weight = weight;
    }

    Segment_2 get_segment(const Edge& e) const {
      return Segment_2(vertices_[e.u].p, vertices_[e.v].p);
    }
};
