#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <string>

#include "CGALTypes.h"

template <class T>
void sort_tuple(T& a, T& b) {
  if (a>b) std::swap(a,b);
}

struct GMLVertexPropertyType {
  std::string x, y;
};

struct GMLEdgePropertyType {
  std::string weight;
  std::string weight_additive;
};

class GMLGraph : public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, GMLVertexPropertyType, GMLEdgePropertyType> {
  private:
    using Base = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, GMLVertexPropertyType, GMLEdgePropertyType>;

    boost::dynamic_properties dp;

  public:
    typedef typename Base::vertex_property_type      vertex_property_type;
    typedef typename Base::vertex_descriptor         vertex_descriptor;

  public:
    GMLGraph()
      : Base()
      , dp(boost::ignore_other_properties)
      {};

    static GMLGraph create_from_graphml(std::istream &istream);

  friend std::ostream& operator<<(std::ostream& os, const GMLGraph& e);
};

