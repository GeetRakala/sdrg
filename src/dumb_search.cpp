//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Finds the minimum among all edge distances and active site node ranges.
 *
 * Iterates through all edges and active nodes in the graph to find the minimum
 * edge distance or active node range. Also provides information on the nodes
 * that make up the minimum edge or the active node.
 *
 * @param bglGraph Reference to the BGLGraph object.
 * @return A tuple containing the node index, range or distance, path if
 * applicable, and a boolean flag indicating if the value is a range.
 */
std::tuple<int, double, std::vector<int>, bool> findMinimumEdgeOrNodeRange( const BGLGraph& bglGraph) {

  //debug bool
  bool debug = false;
  // Initialize variables
  int nodeIndex = -1;
  double minValue = std::numeric_limits<double>::max();
  std::vector<int> path;
  bool isRange = false;

  if (debug) std::cout << "Initialization complete. Starting edge check...\n";

  // Check edge distances
  // Iterate through edges in the graph
  auto edgeRange = boost::edges(bglGraph.getGraph());
  for (auto it = edgeRange.first; it != edgeRange.second; ++it) {
    double edgeWeight = boost::get(boost::edge_weight_t(), bglGraph.getGraph(), *it);

    if (debug) std::cout << "Checking edge between nodes: " << boost::source(*it, bglGraph.getGraph()) << " and " << boost::target(*it, bglGraph.getGraph()) << " with weight: " << edgeWeight << "\n";

    if (edgeWeight < minValue) {
      nodeIndex = static_cast<int>(boost::source(*it, bglGraph.getGraph()));
      minValue = edgeWeight;
      path = {static_cast<int>(boost::target(*it, bglGraph.getGraph())),nodeIndex};
      isRange = false;

      if (debug) std::cout << "Found new minimum edge weight: " << minValue << " between nodes: " << path[0] << " and " << path[1] << "\n";
    }
  }

  if (debug) std::cout << "Edge check complete. Starting active node range check...\n";

  // Check active node ranges
  auto activeNodes = bglGraph.getNodesByStatus().find(NodeStatus::Active);
  if (activeNodes != bglGraph.getNodesByStatus().end()) {
    for (const auto& idx : activeNodes->second) {
      const Node& nodeData = bglGraph.getGraph()[idx];

      if (debug) std::cout << "Checking active node with index: " << idx << " and range: " << nodeData.range << "\n";

      if (nodeData.range < minValue) {
        nodeIndex = idx;
        minValue = nodeData.range;
        path = {};
        isRange = true;

        if (debug) std::cout << "Found new minimum node range: " << minValue << " for node index: " << nodeIndex << "\n";
      }
    }
  }

  if (debug) std::cout << "Active node range check complete. Preparing to return results...\n";

  return std::make_tuple(nodeIndex, minValue, path, isRange);
}
