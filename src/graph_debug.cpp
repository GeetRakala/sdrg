//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Finds and prints all duplicate edges in the graph.
 *
 * @param graph An instance of BGLGraph representing the graph.
 */
void findAndPrintDuplicateEdges(const BGLGraph& graph) {
  const Graph& g = graph.getGraph();
  std::set<std::pair<int, int>> unique_edges;
  std::set<std::pair<int, int>> duplicate_edges;

  boost::graph_traits<Graph>::edge_iterator ei, eend;
  for (boost::tie(ei, eend) = edges(g); ei != eend; ++ei) {
    int source = boost::source(*ei, g);
    int target = boost::target(*ei, g);

    // Make edge directions irrelevant by sorting
    std::pair<int, int> edge = std::minmax(source, target);

    // Check for duplicates
    if (unique_edges.find(edge) != unique_edges.end()) {
      duplicate_edges.insert(edge);
    } else {
      unique_edges.insert(edge);
    }
  }

  // Print duplicate edges
  for (const auto& edge : duplicate_edges) {
    std::cout << "Duplicate edge between nodes " << edge.first
              << " and " << edge.second << "\n";
  }
}
