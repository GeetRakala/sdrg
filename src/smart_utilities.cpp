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

  int firstNode = path[0];
  int lastNode = path.back();

  std::vector<double> pathDist(path.size(), 0.0);

  //for (size_t index = 1; index < path.size() - 1; ++index) {
  //}

  double previousDistance = 0.0; // Initialize previous distance
  if (debug) {
    std::cout << "Entering reassignInactiveEdges with newSourceNode: "
      << newSourceNode << "\n";
  }

  // Loop over the path, skipping the first and last node
  for (size_t index = 1; index < path.size() - 1; ++index) {
    auto [_1, dijkDistFirst, _2] = ActivatedDijkstra(*this, path[index], firstNode, true, debug);
    auto [_3, dijkDistLast, _4] = ActivatedDijkstra(*this, path[index], lastNode, true, debug);
    pathDist[index] = std::min(dijkDistFirst,dijkDistLast) ;
    int oldSourceNode = path[index];

    //previousDistance += get(boost::edge_weight_t(), g, edge(path[index - 1], oldSourceNode, g).first);

    if (debug) {
      std::cout << "  Processing node " << oldSourceNode << " in the path.\n";
      std::cout << "  Cumulative distance (previousDistance) updated to: "
        << pathDist[index] << "\n";
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

      double newEdgeWeight = edgeWeight + pathDist[index];
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

void BGLGraph::updateInactiveNeighborEdgeDistances(int sourceNode, int targetNode, bool debug) {

  if(debug) std::cout << "Entering updateInactiveNeighbourEdgeDistances" << "\n";
  // Compute the Dijkstra distance from targetNode to oldSourceNode
  auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, targetNode, sourceNode, true, debug);
  if(debug) std::cout << "Dijkstra weight: " << dijkstraDist << "\n";
  if(debug) std::cout << "between " << targetNode <<" and " << sourceNode << "\n";
  // Update the edge distance using the updateEdgeDistance method
  updateEdgeDistance(sourceNode, targetNode, dijkstraDist);
  if(debug) std::cout << "Edge distance between " << sourceNode << " and " << targetNode << " updated to " << dijkstraDist << "\n";
}


void BGLGraph::updateNeighborEdgeDistancesLastNode(int startNode, bool debug) {
  // Get out-edges of the startNode
  auto outEdgePair = out_edges(startNode, g);
  for (auto it = outEdgePair.first; it != outEdgePair.second; ++it) {
    // Find the target vertex of each edge
    int targetNode = target(*it, g);
    // Check if the target node is active
    //if (g[targetNode].status == NodeStatus::Active) {
    // Compute the Dijkstra distance to the active target node
    if (debug) std::cout << "Running actDjkstra between " << targetNode << " and " << startNode << "\n";
    auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, targetNode,
        startNode, true, debug);
    // Update the edge distance using the updateEdgeDistance method
    updateEdgeDistance(startNode, targetNode, dijkstraDist);
    if (debug) std::cout << "Updated edge distance between " << startNode << " and " << targetNode << " to " << dijkstraDist << "\n";
    //}
  }
}

void BGLGraph::updateFirstNeighborEdgeDistancesLastNode(int lastNode, int firstNode, bool debug) {
  // Get out-edges of the startNode
  auto outEdgePair = out_edges(firstNode, g);
  for (auto it = outEdgePair.first; it != outEdgePair.second; ++it) {
    // Find the target vertex of each edge
    int targetNode = target(*it, g);
    double edgeWeight = get(boost::edge_weight_t(), g, *it);
    // Compute the Dijkstra distance to the target node
    if (debug) std::cout << "Running actDjkstra between " << targetNode << " and " << firstNode << "\n";
    auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, targetNode,
        lastNode, true, debug);
    // Update the edge distance using the updateEdgeDistance method
    updateEdgeDistance(firstNode, targetNode, std::min(edgeWeight,dijkstraDist));
    if (debug) std::cout << "Updated edge distance between " << firstNode << " and " << targetNode << " to " << dijkstraDist << "\n";
    //}
}
}


//void BGLGraph::removeDuplicateEdges(int sourceNode, bool debug) {
//  if(debug) std::cout << "Entering removeDuplicateEdges for sourceNode: " << sourceNode << "\n";
//
//  // Identify minimum weights for unique target nodes
//  std::unordered_map<int, double> uniqueEdges;
//  boost::graph_traits<Graph>::out_edge_iterator ei, eend;
//  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend; ++ei) {
//    int targetNode = target(*ei, g);
//    double edgeWeight = get(boost::edge_weight_t(), g, *ei);
//
//    auto it = uniqueEdges.find(targetNode);
//    if (it != uniqueEdges.end()) {
//      it->second = std::min(it->second, edgeWeight);
//    } else {
//      uniqueEdges[targetNode] = edgeWeight;
//    }
//  }
//
//  // Remove duplicate edges while keeping the ones with minimum weight
//  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend;) {
//    int targetNode = target(*ei, g);
//    double edgeWeight = get(boost::edge_weight_t(), g, *ei);
//
//    if (edgeWeight > uniqueEdges[targetNode]) {
//      // Remove edges that are not minimum
//      remove_edge(*ei++, g);
//      if(debug) std::cout << "Removed edge to: " << targetNode << "\n";
//    } else {
//      ++ei;
//    }
//  }
//
//  // Additional step to remove one of the bidirectional edges with the same weight
//  //for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend; ++ei) {
//  //  int targetNode = target(*ei, g);
//  //  if (edge(targetNode, sourceNode, g).second) {
//  //    removeEdge(targetNode, sourceNode);
//  //    break;  // Exit the loop once we remove one such edge
//  //  }
//  //}
//
//  // Additional step to remove one of the bidirectional edges with the same weight
//  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend; ++ei) {
//    int targetNode = target(*ei, g);
//    if (edge(targetNode, sourceNode, g).second) {
//      removeEdge(sourceNode, targetNode);
//      if(debug) std::cout << "Removed one bidirectional edge to: " << targetNode << "\n";
//      break;
//    }
//  }
//
//  if(debug) std::cout << "Exiting removeDuplicateEdges for sourceNode: " << sourceNode << "\n";
//}

/**
 * @brief Removes duplicate edges originating from a source node and keeps the
 * edge with the minimum weight for each source->target pair.
 *
 * @param sourceNode The source node from which to remove duplicate edges.
 * @param g The graph object.
 * @param debug Flag to print debug information.
 */
void BGLGraph::removeDuplicateEdges(int sourceNode, bool debug) {
  using EdgeDesc = typename boost::graph_traits<Graph>::edge_descriptor;
  std::unordered_map<int, EdgeDesc> best_edge;
  std::vector<EdgeDesc> edges_to_remove;

  // Debug: Print edges before operation
  if (debug) {
    std::cout << "Edges before operation:\n";
    auto [ei, eend] = out_edges(sourceNode, g);
    for (; ei != eend; ++ei) {
      int targetNode = target(*ei, g);
      double edgeWeight = get(boost::edge_weight, g)[*ei];
      std::cout << sourceNode << " -> " << targetNode << " (weight: " << edgeWeight << ")\n";
    }
  }

  // Step 1: Identify duplicate edges and decide which to keep
  auto [ei, eend] = out_edges(sourceNode, g);
  for (; ei != eend; ++ei) {
    int targetNode = target(*ei, g);
    double edgeWeight = get(boost::edge_weight, g)[*ei];

    if (best_edge.find(targetNode) == best_edge.end()) {
      best_edge[targetNode] = *ei;
    } else {
      double bestWeight = get(boost::edge_weight, g)[best_edge[targetNode]];
      if (edgeWeight < bestWeight) {
        edges_to_remove.push_back(best_edge[targetNode]);
        best_edge[targetNode] = *ei;
      } else {
        edges_to_remove.push_back(*ei);
      }
    }
  }

  // Step 2: Remove duplicate edges
  for (const auto& edge : edges_to_remove) {
    remove_edge(edge, g);
  }

  // Debug: Print edges after operation
  if (debug) {
    std::cout << "Edges after operation:\n";
    auto [ei, eend] = out_edges(sourceNode, g);
    for (; ei != eend; ++ei) {
      int targetNode = target(*ei, g);
      double edgeWeight = get(boost::edge_weight, g)[*ei];
      std::cout << sourceNode << " -> " << targetNode << " (weight: " << edgeWeight << ")\n";
    }
  }
}

void BGLGraph::updateAllEdgeDistancesFromNode(int node_index,
    bool debug,
    bool debugProcessNegativeEdge) {
  // Calculate half of the range for the current node
  double half_range = g[node_index].range / 2.0;
  if (debug) {
    std::cout << "Half range for node " << node_index << ": " << half_range << "\n";
  }

  // Container for storing negative edges
  std::vector<std::pair<EdgeDescriptor, double>> negative_edges;

  boost::graph_traits<Graph>::out_edge_iterator out_ei, out_eend;

  // Iterate through each edge originating from the node
  for (boost::tie(out_ei, out_eend) = out_edges(node_index, g); out_ei != out_eend; ++out_ei) {
    int target_node = boost::target(*out_ei, g);

    // Update distance for each edge and check for negative distances
    double old_distance = get(boost::edge_weight_t(), g, *out_ei);
    double new_distance = old_distance - half_range;
    if (new_distance < 0) {
      negative_edges.emplace_back(*out_ei, new_distance);
    }
    if (debug) {
      std::cout << "Updated edge from " << node_index << " to " << target_node;
      std::cout << ". Old Distance: " << old_distance << ", New Distance: " << new_distance << "\n";
    }
    updateEdgeDistance(node_index, target_node, new_distance);
  }

  // Keep track of newly created negative edges
  std::vector<std::pair<EdgeDescriptor, double>> newly_negative_edges;


  //while (!negative_edges.empty() || !newly_negative_edges.empty()) {
  bool keepRunning = true;
  int loopCount = 0;  // Counter to trace the number of loop iterations

  // Loop continues until no more negative edges are created or processed.
  while (keepRunning) {
    // Assume we won't continue unless we find new negative edges.
    keepRunning = false;
    loopCount++;

    if (debug) {
      std::cout << "Processing negative edges..." << "\n";
    }

    for (const auto& pair : negative_edges) {
      if (debug) {
        std::cout << "Current negative edge: " << pair.first << ", " << pair.second << "\n";
      }
      processNegativeEdge(pair.first, pair.second, debugProcessNegativeEdge, newly_negative_edges);
      keepRunning = true;
    }
    negative_edges.clear();  // Clear for the next iteration

    if (debug) {
      std::cout << "Processing newly created negative edges..." << "\n";
    }

    // Logic to keep only minimum distance duplicates among newly created negative edges
    std::sort(newly_negative_edges.begin(), newly_negative_edges.end());
    auto it = std::unique(newly_negative_edges.begin(), newly_negative_edges.end(),
        [](const auto& a, const auto& b) { return a.first == b.first; });
    newly_negative_edges.erase(it, newly_negative_edges.end());

    for (const auto& pair : newly_negative_edges) {
      if (debug) {
        std::cout << "Newly created negative edge: " << pair.first << ", " << pair.second << "\n";
      }
      processNegativeEdge(pair.first, pair.second, debug, negative_edges);
      keepRunning = true;
      //std::cout << "Waiting for getchar." << "\n";
      //getchar();

    }

    newly_negative_edges.clear();  // Clear for the next iteration

    if (debug) {
      std::cout << "End of one iteration in the while loop." << "\n";
    }

    // Sanity check to break out of a potential infinite loop
    if (loopCount > 1000) {
      std::cerr << "Warning: Exceeded 1000 iterations, breaking out of loop.\n";
      break;
    }

  }
  //}

  if (debug) {
    std::cout << "Updated edge distances originating from node: " << node_index << "\n";
  }

}

void BGLGraph::processNegativeEdge(EdgeDescriptor edgeWithNegativeDistance,
    double negativeEdgeDistance,
    bool debug,
    std::vector<std::pair<EdgeDescriptor, double>>& newly_negative_edges) {

  int sourceNode = boost::source(edgeWithNegativeDistance, g);
  int targetNode = boost::target(edgeWithNegativeDistance, g);

  // Re-fetch the edge distance between sourceNode and targetNode
  double newFetchedDistance = get(boost::edge_weight_t(), g, edgeWithNegativeDistance);
  // Update negativeEdgeDistance to the minimum value
  negativeEdgeDistance = std::min(negativeEdgeDistance, newFetchedDistance);


  bool debug1=false;

  if (debug) {
    std::cout << "Processing negative edge from " << sourceNode << " to " << targetNode;
    std::cout << " with distance: " << negativeEdgeDistance << "\n";
  }

  boost::graph_traits<Graph>::out_edge_iterator neighborEdgeIter, neighborEdgeEnd;


  // Start of loop to iterate through each out-edge of the target node
  if (debug) {
    std::cout << "Starting iteration through out-edges of node "
      << targetNode << "." << "\n";
  }

  // Iterate through each out-edge of the target node
  for (boost::tie(neighborEdgeIter, neighborEdgeEnd) = out_edges(targetNode, g);
      neighborEdgeIter != neighborEdgeEnd; ++neighborEdgeIter) {

    // Get the target node of the current edge being examined
    int neighborNode = boost::target(*neighborEdgeIter, g);

    if (debug1) {
      std::cout << "Currently examining out-edge from " << targetNode
        << " to " << neighborNode << "." << "\n";
    }

    // Check if the current edge is a back edge to the original source node
    if (neighborNode == sourceNode) {
      if (debug1) {
        std::cout << "Skipping edge as it's a back edge to the original source node "
          << sourceNode << "." << "\n";
      }
      continue;
    }

    // Retrieve the existing distance for the current edge
    double distanceFromTargetToNeighbor = get(boost::edge_weight_t(), g, *neighborEdgeIter);

    // Calculate the new distance for the edge using the distance of the negative edge
    double newEdgeDistance = negativeEdgeDistance + distanceFromTargetToNeighbor;

    if (debug1) {
      std::cout << "Calculating new edge distance from " << sourceNode << " to " << neighborNode;
      std::cout << ". New Distance: " << newEdgeDistance << "\n";
    }

    // Check if an edge already exists between the source node and the neighbor node
    EdgeDescriptor existingEdgeDescriptor;
    bool edgeAlreadyExists;
    boost::tie(existingEdgeDescriptor, edgeAlreadyExists) = edge(sourceNode, neighborNode, g);

    if (debug) {
      std::cout << "Checking if an edge already exists from " << sourceNode << " to "
        << neighborNode << ". Result: " << std::to_string(edgeAlreadyExists) << "\n";
    }

    // Update the edge if it already exists, or add a new edge
    if (edgeAlreadyExists) {
      // Get the existing edge distance
      double existingEdgeDistance = get(boost::edge_weight_t(), g, existingEdgeDescriptor);

      // Update the edge distance to the minimum of existing and new distance
      updateEdgeDistance(sourceNode, neighborNode, std::min(existingEdgeDistance, newEdgeDistance));

      double updatedEdgeDistance = get(boost::edge_weight_t(), g, existingEdgeDescriptor);

      // If the updated edge distance is negative, add it to the list of newly created negative edges
      if (updatedEdgeDistance < 0) {
        newly_negative_edges.emplace_back(existingEdgeDescriptor, updatedEdgeDistance);
      }

      if (debug) {
        std::cout << "Updated existing edge from " << sourceNode << " to " << neighborNode;
        std::cout << ". Updated Distance: " << updatedEdgeDistance << "\n";
      }
    }
    else {
      // Add a new edge with the calculated new distance
      addEdge(sourceNode, neighborNode, newEdgeDistance);

      // If the new edge distance is negative, add it to the list of newly created negative edges
      if (newEdgeDistance < 0) {
        newly_negative_edges.emplace_back(edge(sourceNode, neighborNode, g).first, newEdgeDistance);
      }

      if (debug) {
        std::cout << "Added new edge from " << sourceNode << " to " << neighborNode;
        std::cout << ". New Distance: " << newEdgeDistance << "\n";
      }
    }
  }

  // End of loop to iterate through each out-edge of the target node
  if (debug) {
    std::cout << "Ended iteration through out-edges of node " << targetNode << "." << "\n";
  }


  // Remove all outgoing and incoming edges for the target node
  removeOutEdges(targetNode);
  removeInEdges(targetNode);


  if (debug) {
    std::cout << "Target node added for edge removal " << targetNode << "\n";
  }

}
