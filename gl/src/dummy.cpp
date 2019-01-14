#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
  
using Kernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using Point_2 = typename Kernel::Point_2;
using Line_2 = typename Kernel::Line_2;

#include "gml/GMLGraph.h"

#include <fstream>
#include <iostream>

int main(int,char*[]) {
  Point_2 u(0,1);
  Point_2 v(1,1);
  Line_2 l(u,v);
  std::cout << l << std::endl;

  GMLGraph graph;

  //std::ifstream dot("../x");
  graph = GMLGraph::create_from_graphml(std::cin);
  // graph.print();


//  SkeletonStructure s;
//  s.add_graph(graph);
//  s.initialize();

  return 0;
}
