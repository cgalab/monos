/**
 *  Copyright 2018, 2019 Peter Palfraader
 *		            2019 Günther Eder - geder@cs.sbg.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
#ifdef WITH_FP
		Point p(std::stod(graph[v].x), std::stod(graph[v].y));
#else
		Point p(graph[v].x, graph[v].y);
#endif
		unsigned degree = boost::degree(v, graph);
		add_vertex(Vertex(p, vertices_.size()));
		assert(index_map[v] == vertices_.size()-1);
		if (degree == 1) num_of_deg1_vertices++;
	}
	std::vector<std::tuple<sl,sl>> edgePairs;
	for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
		const EdgeType e = *ep.first;
		edgePairs.emplace_back(std::make_tuple( (sl)source(e, graph), (sl)target(e, graph) ));
	}

	std::sort(edgePairs.begin(),edgePairs.end());

	std::vector<sl> map(vertices_.size(), NIL);
	for(const auto& e : edgePairs) {
		LOG(INFO) << std::get<0>(e) << " " <<  std::get<1>(e);
		if(map[std::get<0>(e)] == NIL) {
			map[std::get<0>(e)] = std::get<1>(e);
		} else if(map[std::get<1>(e)] == NIL) {
			map[std::get<1>(e)] = std::get<0>(e);
		} else {
			LOG(WARNING) << "additional edge " << std::get<0>(e) << ", "<< std::get<1>(e);
		}
	}

	unsigned idx = 0;
	do {
		add_edge(idx,map[idx]);
		idx = map[idx];
	} while(idx != 0);
}
