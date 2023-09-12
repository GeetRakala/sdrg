//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Updates edges between all connected nodes of a given node and removes
 * the node.
 *
 * This function identifies all the connected nodes (neighbors) of the input
 * node 'inputNode'. For each unique pair of neighbors (neighborA, neighborB),
 * it calculates and updates the edge weight based on the formula:
 * weight_to_neighborA + weight_to_neighborB - Range(inputNode).
 * Finally, it removes all edges connected to 'inputNode'.
 *
 * @param inputNode The node whose neighbors need to be connected.
 * @param debug A boolean flag to enable debug logs.
 */
void BGLGraph::updateEdgesAndRemoveNode(int inputNode, bool debug) {
  // Debug log for entering the function
  if (debug) {
    std::cout << "Entering updateEdgesAndRemoveNode for node: "
              << inputNode << "\n";
  }

  // Step 1: Collect all neighbors of the input node
  std::vector<int> neighborNodes;
  boost::graph_traits<Graph>::out_edge_iterator edgeIt, edgeEnd;
  for (boost::tie(edgeIt, edgeEnd) = out_edges(inputNode, g);
       edgeIt != edgeEnd; ++edgeIt) {
    int targetNode = boost::target(*edgeIt, g);
    neighborNodes.push_back(targetNode);
  }

  // Step 2: Create new edges between every unique pair of neighbors
  double inputNodeRange = g[inputNode].range;  // Get the range of the input node
  for (size_t i = 0; i < neighborNodes.size(); ++i) {
    int neighborA = neighborNodes[i];
    for (size_t j = i + 1; j < neighborNodes.size(); ++j) {
      int neighborB = neighborNodes[j];

      // Calculate the new edge weight
      double weightToNeighborA = get(boost::edge_weight_t(),
                                     g, edge(inputNode, neighborA, g).first);
      double weightToNeighborB = get(boost::edge_weight_t(),
                                     g, edge(inputNode, neighborB, g).first);
      double newEdgeWeight = weightToNeighborA + weightToNeighborB
                             - inputNodeRange;

      // Add the new edge with updated weight
      addEdge(neighborA, neighborB, newEdgeWeight);

      // Debug log for new edge addition
      if (debug) {
        std::cout << "  Added edge between " << neighborA << " and "
                  << neighborB << " with updated weight: " << newEdgeWeight
                  << "\n";
      }
    }
  }
  for (size_t i = 0; i < neighborNodes.size(); ++i) {
    int neighbor = neighborNodes[i];
    removeDuplicateEdges(neighbor,debug);
  }

  // Step 3: Remove all outgoing edges from the input node
  removeOutEdges(inputNode);

  // Debug log for edge removal
  if (debug) {
    std::cout << "  Removed all edges connected to node " << inputNode << "\n";
  }
}
