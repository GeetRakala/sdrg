//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"

void BGLGraph::removeSelfLoops(int sourceNode, bool debug) {
  if(debug) std::cout << "Entering removeSelfLoops for sourceNode: " << sourceNode << "\n";
  boost::graph_traits<Graph>::out_edge_iterator ei, eend;
  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend;) {
    int targetNode = boost::target(*ei, g); // Get the target node
    if (targetNode == sourceNode) {
      int from = boost::source(*ei, g);
      int to = targetNode;
      if(debug) std::cout << "  Removing self-loop from " << from << " to " << to << "\n";
      removeEdge(from, to); // Remove self-loop using the removeEdge method
      boost::tie(ei, eend) = out_edges(sourceNode, g); // Refresh the iterators
    } else {
      ++ei; // Move to next edge
    }
  }
}

/**
 * @brief Reassign inactive edges from nodes in the path to a new source node.
 *
 * @param newSourceNode The new node to which edges will be reassigned.
 * @param path The path of nodes whose edges are to be reassigned.
 * @param debug Flag to control debug output.
 */
void BGLGraph::reassignInactiveEdges(int newSourceNode, const std::vector<int>& path, bool debug) {
  // Container for edges to remove
  std::vector<std::pair<int, int>> edgesToRemove;

  double previousDistance = 0.0; // Initialize previous distance
  if (debug) {
    std::cout << "Entering reassignInactiveEdges with newSourceNode: "
      << newSourceNode << "\n";
  }

  // Loop over the path, skipping the first and last node
  for (size_t index = 1; index < path.size() - 1; ++index) {
    int oldSourceNode = path[index];
    previousDistance += get(boost::edge_weight_t(), g,
        edge(path[index - 1], oldSourceNode, g).first);

    if (debug) {
      std::cout << "  Processing node " << oldSourceNode << " in the path.\n";
      std::cout << "  Cumulative distance (previousDistance) updated to: "
        << previousDistance << "\n";
    }

    // Process outgoing edges
    boost::graph_traits<Graph>::out_edge_iterator out_ei, out_eend;
    for (boost::tie(out_ei, out_eend) = out_edges(oldSourceNode, g);
        out_ei != out_eend; ++out_ei) {
      int targetNode = boost::target(*out_ei, g);
      double edgeWeight = get(boost::edge_weight_t(), g, *out_ei);

      if (debug) {
        std::cout << "    Processing outgoing edge from " << oldSourceNode
          << " to " << targetNode << " with original weight: "
          << edgeWeight << "\n";
      }

      double newEdgeWeight = edgeWeight + previousDistance;
      addEdge(newSourceNode, targetNode, newEdgeWeight);
      edgesToRemove.push_back({oldSourceNode, targetNode});

      if (debug) {
        std::cout << "      Added edge from " << newSourceNode << " to "
          << targetNode << " with updated weight: " << newEdgeWeight << "\n";
      }
    }
  }

  // Remove all the edges that were marked for removal
  for (const auto& edge : edgesToRemove) {
    removeEdge(edge.first, edge.second);
    if (debug) {
      std::cout << "      Removed edge from " << edge.first << " to "
        << edge.second << "\n";
    }
  }

  if (debug) {
    std::cout << "Exiting reassignInactiveEdges.\n";
  }
}


void BGLGraph::reassignEdges(int newSourceNode, int oldSourceNode, bool debug) {
  if(debug) std::cout << "Entering reassignEdges with newSourceNode: " << newSourceNode << ", oldSourceNode: " << oldSourceNode << "\n";
  // Process outgoing edges
  boost::graph_traits<Graph>::out_edge_iterator out_ei, out_eend;
  // Iterate over all outgoing edges from the old source node
  for (boost::tie(out_ei, out_eend) = out_edges(oldSourceNode, g); out_ei != out_eend;) {
    int targetNode = boost::target(*out_ei, g);
    double edgeWeight = get(boost::edge_weight_t(), g, *out_ei);
    if(debug) std::cout << "  Processing outgoing edge from " << oldSourceNode << " to " << targetNode << " with weight: " << edgeWeight << "\n";
    // Reassign the edge to the new source node
    addEdge(newSourceNode, targetNode, edgeWeight);
    if(debug) std::cout << "    Added edge from " << newSourceNode << " to " << targetNode << " with weight: " << edgeWeight << "\n";
    removeEdge(oldSourceNode, targetNode);
    if(debug) std::cout << "    Removed edge from " << oldSourceNode << " to " << targetNode << "\n";
    // Refresh the iterators
    boost::tie(out_ei, out_eend) = out_edges(oldSourceNode, g);
  }
}

void BGLGraph::updateActiveNeighborEdgeDistances(int startNode, bool debug) {
  // Get out-edges of the startNode
  auto outEdgePair = out_edges(startNode, g);
  for (auto it = outEdgePair.first; it != outEdgePair.second; ++it) {
    // Find the target vertex of each edge
    int targetNode = target(*it, g);
    // Check if the target node is active
    if (g[targetNode].status == NodeStatus::Active) {
      // Compute the Dijkstra distance to the active target node
      auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, startNode,
          targetNode, true, debug);
      // Update the edge distance using the updateEdgeDistance method
      updateEdgeDistance(startNode, targetNode, dijkstraDist);
    }
  }
}

void BGLGraph::removeDuplicateEdges(int sourceNode, bool debug) {
  if(debug) std::cout << "Entering removeDuplicateEdges for sourceNode: " << sourceNode << "\n";

  // Identify minimum weights for unique target nodes
  std::unordered_map<int, double> uniqueEdges;
  boost::graph_traits<Graph>::out_edge_iterator ei, eend;
  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend; ++ei) {
    int targetNode = target(*ei, g);
    double edgeWeight = get(boost::edge_weight_t(), g, *ei);

    auto it = uniqueEdges.find(targetNode);
    if (it != uniqueEdges.end()) {
      it->second = std::min(it->second, edgeWeight);
    } else {
      uniqueEdges[targetNode] = edgeWeight;
    }
  }

  // Remove duplicate edges while keeping the ones with minimum weight
  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend;) {
    int targetNode = target(*ei, g);
    double edgeWeight = get(boost::edge_weight_t(), g, *ei);

    if (edgeWeight > uniqueEdges[targetNode]) {
      // Remove edges that are not minimum
      remove_edge(*ei++, g);
      if(debug) std::cout << "Removed edge to: " << targetNode << "\n";
    } else {
      ++ei;
    }
  }

  if(debug) std::cout << "Exiting removeDuplicateEdges for sourceNode: " << sourceNode << "\n";
}


void BGLGraph::updateAllEdgeDistancesFromNode(int node_index, bool debug, bool debugProcessNegativeEdge) {
  if(debug) std::cout << "Updating all edge distances from node " << node_index << std::endl;
  double half_range = g[node_index].range / 2.0;
  std::vector<std::pair<EdgeDescriptor, double>> negative_edges;
  boost::graph_traits<Graph>::out_edge_iterator out_ei, out_eend;
  for (boost::tie(out_ei, out_eend) = out_edges(node_index, g); out_ei != out_eend; ++out_ei) {
    int target_node = boost::target(*out_ei, g);
    double old_distance = get(boost::edge_weight_t(), g, *out_ei);
    double new_distance = old_distance - half_range;
    if(debug) std::cout << "Evaluating edge: " << node_index << " -> " << target_node << ", distance: " << new_distance << std::endl;
    if (new_distance < 0) {
      if(debug) std::cout << "Negative edge distance found. Storing for later processing." << std::endl;
      negative_edges.emplace_back(*out_ei, new_distance);
      continue;
    }
    if(debug) std::cout << "Updating edge distance: " << node_index << " -> " << target_node << std::endl;
    updateEdgeDistance(node_index, target_node, new_distance);
  }
  if(debug) std::cout << "Processing negative edges..." << std::endl;
  for (const auto& pair : negative_edges) {
    processNegativeEdge(pair.first, pair.second, debugProcessNegativeEdge);
  }
  // Final check
  if(debug) std::cout << "Final check for remaining negative edges." << std::endl;
  for (boost::tie(out_ei, out_eend) = out_edges(node_index, g); out_ei != out_eend; ++out_ei) {
    double updated_distance = get(boost::edge_weight_t(), g, *out_ei);
    if (updated_distance < 0) {
      if(debug) std::cout << "Exception: Remaining negative edge found after processing." << std::endl;
      throw std::runtime_error("Remaining negative edge found after processing.");
    }
  }
}

void BGLGraph::processNegativeEdge(EdgeDescriptor edgeWithNegativeDistance, double negativeEdgeDistance, bool debug) {
  if(debug) std::cout << "Processing negative edge..." << std::endl;
  int sourceNode = boost::source(edgeWithNegativeDistance, g);
  int targetNode = boost::target(edgeWithNegativeDistance, g);
  if(debug) std::cout << "Negative edge: " << sourceNode << " -> " << targetNode << ", distance: " << negativeEdgeDistance << std::endl;
  boost::graph_traits<Graph>::out_edge_iterator neighborEdgeIter, neighborEdgeEnd;
  for (boost::tie(neighborEdgeIter, neighborEdgeEnd) = out_edges(targetNode, g);
      neighborEdgeIter != neighborEdgeEnd; ++neighborEdgeIter) {
    int neighborNode = boost::target(*neighborEdgeIter, g);
    if(debug) std::cout << "Evaluating neighbor: " << neighborNode << std::endl;
    if (neighborNode == sourceNode) continue;
    double distanceFromTargetToNeighbor = get(boost::edge_weight_t(), g, *neighborEdgeIter);
    if(debug) std::cout << "Distance from target node " << targetNode << " to neighbor node " << neighborNode << " is " << distanceFromTargetToNeighbor << std::endl;
    double newEdgeDistance = negativeEdgeDistance + distanceFromTargetToNeighbor;
    newEdgeDistance = std::max(0.0, newEdgeDistance);  // Make it zero if it's negative  // IS THIS OK??!!
    //if (newEdgeDistance == 0) std::cout << "Edge has been reweighted to zero" << std::endl;
    if(debug) std::cout << "New edge distance from source node " << sourceNode << " to neighbor node " << neighborNode << " is " << newEdgeDistance << std::endl;
    if(debug) std::cout << "New calculated edge distance: " << sourceNode << " -> " << neighborNode << " is " << newEdgeDistance << std::endl;
    EdgeDescriptor existingEdgeDescriptor;
    bool edgeAlreadyExists;
    boost::tie(existingEdgeDescriptor, edgeAlreadyExists) = edge(sourceNode, neighborNode, g);
    if (edgeAlreadyExists) {
      if(debug) std::cout << "Edge already exists. Updating..." << std::endl;
      double existingEdgeDistance = get(boost::edge_weight_t(), g, existingEdgeDescriptor);
      if(debug) std::cout << "Existing edge distance: " << existingEdgeDistance << std::endl;
      updateEdgeDistance(sourceNode, neighborNode, std::min(existingEdgeDistance, newEdgeDistance));
      double updatedEdgeDistance = get(boost::edge_weight_t(), g, existingEdgeDescriptor);
      if(debug) std::cout << "Updated edge distance: " << updatedEdgeDistance << std::endl;
    } else {
      if(debug) std::cout << "Edge does not exist. Adding..." << std::endl;
      addEdge(sourceNode, neighborNode, newEdgeDistance);
    }
  }
  if(debug) std::cout << "Removing all out-edges and in-edges of the target node " << targetNode << std::endl;
  removeOutEdges(targetNode);
  removeInEdges(targetNode);
}
