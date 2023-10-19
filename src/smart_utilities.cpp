//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"

void BGLGraph::removeSelfLoops(int sourceNode, bool debug) {
  boost::graph_traits<Graph>::out_edge_iterator ei, eend;
  for (boost::tie(ei, eend) = out_edges(sourceNode, g); ei != eend;) {
    int targetNode = boost::target(*ei, g); // Get the target node
    if (targetNode == sourceNode) {
      int from = boost::source(*ei, g);
      int to = targetNode;
      removeEdge(from, to); // Remove self-loop using the removeEdge method
      boost::tie(ei, eend) = out_edges(sourceNode, g); // Refresh the iterators
    } else {
      ++ei; // Move to next edge
    }
  }
}

void BGLGraph::reassignInactiveEdges(int newSourceNode, const std::vector<int>& path, bool debug) {
  // Container for edges to remove
  std::vector<std::pair<int, int>> edgesToRemove;
  int firstNode = path[0];
  int lastNode = path.back();
  std::vector<double> pathDist(path.size(), 0.0);
  double previousDistance = 0.0; // Initialize previous distance
                                 // Loop over the path, skipping the first and last node
  for (size_t index = 1; index < path.size() - 1; ++index) {
    auto [_1, dijkDistFirst, _2] = ActivatedDijkstra(*this, path[index], firstNode, true, debug);
    auto [_3, dijkDistLast, _4] = ActivatedDijkstra(*this, path[index], lastNode, true, debug);
    pathDist[index] = std::min(dijkDistFirst,dijkDistLast) ;
    int oldSourceNode = path[index];
    // Process outgoing edges
    boost::graph_traits<Graph>::out_edge_iterator out_ei, out_eend;
    for (boost::tie(out_ei, out_eend) = out_edges(oldSourceNode, g);
        out_ei != out_eend; ++out_ei) {
      int targetNode = boost::target(*out_ei, g);
      double edgeWeight = get(boost::edge_weight_t(), g, *out_ei);
      double newEdgeWeight = edgeWeight + pathDist[index];
      addEdge(newSourceNode, targetNode, newEdgeWeight);
      edgesToRemove.push_back({oldSourceNode, targetNode});
    }
  }
  // Remove all the edges that were marked for removal
  for (const auto& edge : edgesToRemove) {
    removeEdge(edge.first, edge.second);
  }
}


void BGLGraph::reassignEdges(int newSourceNode, int oldSourceNode, bool debug) {
  // Process outgoing edges
  boost::graph_traits<Graph>::out_edge_iterator out_ei, out_eend;
  // Iterate over all outgoing edges from the old source node
  for (boost::tie(out_ei, out_eend) = out_edges(oldSourceNode, g); out_ei != out_eend;) {
    int targetNode = boost::target(*out_ei, g);
    double edgeWeight = get(boost::edge_weight_t(), g, *out_ei);
    // Reassign the edge to the new source node
    addEdge(newSourceNode, targetNode, edgeWeight);
    removeEdge(oldSourceNode, targetNode);
    // Refresh the iterators
    boost::tie(out_ei, out_eend) = out_edges(oldSourceNode, g);
  }
}

void BGLGraph::updateInactiveNeighborEdgeDistances(int sourceNode, int targetNode, bool debug) {
  // Compute the Dijkstra distance from targetNode to oldSourceNode
  auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, targetNode, sourceNode, true, debug);
  // Update the edge distance using the updateEdgeDistance method
  updateEdgeDistance(sourceNode, targetNode, dijkstraDist);
}


void BGLGraph::updateNeighborEdgeDistancesLastNode(int startNode, bool debug) {
  // Get out-edges of the startNode
  auto outEdgePair = out_edges(startNode, g);
  for (auto it = outEdgePair.first; it != outEdgePair.second; ++it) {
    // Find the target vertex of each edge
    int targetNode = target(*it, g);
    // Compute the Dijkstra distance to the active target node
    auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, targetNode,
        startNode, true, debug);
    // Update the edge distance using the updateEdgeDistance method
    updateEdgeDistance(startNode, targetNode, dijkstraDist);
  }
}

void BGLGraph::updateFirstNeighborEdgeDistancesLastNode(int lastNode, int firstNode, bool debug) {
  auto outEdgePair = out_edges(firstNode, g);
  for (auto it = outEdgePair.first; it != outEdgePair.second; ++it) {
    int targetNode = target(*it, g);
    double edgeWeight = get(boost::edge_weight_t(), g, *it);
    auto [endNode, dijkstraDist, path] = ActivatedDijkstra(*this, targetNode,
        lastNode, true, debug);
    // Update the edge distance using the updateEdgeDistance method
    updateEdgeDistance(firstNode, targetNode, std::min(edgeWeight,dijkstraDist));
  }
}


void BGLGraph::removeDuplicateEdges(int sourceNode, bool debug) {
  using EdgeDesc = typename boost::graph_traits<Graph>::edge_descriptor;
  std::unordered_map<int, EdgeDesc> best_edge;
  std::vector<EdgeDesc> edges_to_remove;
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
}


void BGLGraph::updateAllEdgeDistancesFromNode(int node_index,
    bool debug,
    bool debugProcessNegativeEdge) {
  // Calculate half of the range for the current node
  double half_range = g[node_index].range / 2.0;
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
    updateEdgeDistance(node_index, target_node, new_distance);
  }

  // Container for newly created negative edges
  std::vector<std::pair<EdgeDescriptor, double>> newly_negative_edges;
  bool keepRunning = true;
  int loopCount = 0;  // Counter to trace the number of loop iterations
  // Loop continues until no more negative edges are created or processed.
  while (keepRunning) {
    // Assume we won't continue unless we find new negative edges.
    keepRunning = false;
    loopCount++;

    for (const auto& pair : negative_edges) {
      processNegativeEdge(pair.first, pair.second, debugProcessNegativeEdge, newly_negative_edges);
      keepRunning = true;
    }
    negative_edges.clear();  // Clear for the next iteration

    for (const auto& pair : newly_negative_edges) {
      processNegativeEdge(pair.first, pair.second, debug, negative_edges);
      keepRunning = true;
    }
    newly_negative_edges.clear();  // Clear for the next iteration

    // Sanity check to break out of a potential infinite loop
    if (loopCount > 1000) {
      std::cerr << "Warning: Exceeded 1000 iterations, breaking out of loop.\n";
      break;
    }
  }
}

void BGLGraph::processNegativeEdge(EdgeDescriptor edgeWithNegativeDistance,
    double negativeEdgeDistance,
    bool debug,
    std::vector<std::pair<EdgeDescriptor, double>>& newly_negative_edges) {
  int sourceNode = boost::source(edgeWithNegativeDistance, g);
  int targetNode = boost::target(edgeWithNegativeDistance, g);

  // Ensure negativeEdgeDistance is updated to the minimal value
  negativeEdgeDistance = std::min(negativeEdgeDistance, get(boost::edge_weight_t(), g, edgeWithNegativeDistance));

  boost::graph_traits<Graph>::out_edge_iterator neighborEdgeIter, neighborEdgeEnd;

  // Iterate through out-edges of the target node
  for (boost::tie(neighborEdgeIter, neighborEdgeEnd) = out_edges(targetNode, g);
      neighborEdgeIter != neighborEdgeEnd; ++neighborEdgeIter) {
    int neighborNode = boost::target(*neighborEdgeIter, g);

    // Skip back edges to the original source node
    if (neighborNode == sourceNode) continue;

    double distanceFromTargetToNeighbor = get(boost::edge_weight_t(), g, *neighborEdgeIter);
    double newEdgeDistance = negativeEdgeDistance + distanceFromTargetToNeighbor;

    EdgeDescriptor existingEdgeDescriptor;
    bool edgeAlreadyExists;
    boost::tie(existingEdgeDescriptor, edgeAlreadyExists) = edge(sourceNode, neighborNode, g);

    if (edgeAlreadyExists) {
      double existingEdgeDistance = get(boost::edge_weight_t(), g, existingEdgeDescriptor);
      double updatedEdgeDistance = std::min(existingEdgeDistance, newEdgeDistance);

      updateEdgeDistance(sourceNode, neighborNode, updatedEdgeDistance);

      if (updatedEdgeDistance < 0) {
        newly_negative_edges.emplace_back(existingEdgeDescriptor, updatedEdgeDistance);
      }
    } else {
      addEdge(sourceNode, neighborNode, newEdgeDistance);

      if (newEdgeDistance < 0) {
        newly_negative_edges.emplace_back(edge(sourceNode, neighborNode, g).first, newEdgeDistance);
      }
    }
  }

  // Remove all outgoing and incoming edges for the target node
  removeOutEdges(targetNode);
  removeInEdges(targetNode);
}
