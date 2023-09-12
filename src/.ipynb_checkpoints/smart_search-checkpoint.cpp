//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Chooses a random active start node from the graph.
 *
 * @param graph Reference to the BGLGraph object containing the nodes and their statuses.
 * @param gen Reference to a std::mt19937 random number generator.
 * @return Index of the randomly selected active node, or -1 if no active nodes are found.
 */
int chooseRandomActiveStartNode(const BGLGraph& graph, std::mt19937& gen) {
  // Use the accessor method to get nodes by status
  auto nodes_by_status = graph.getNodesByStatus();
  // Use find method to get an iterator pointing to the active nodes in the graph
  auto it = nodes_by_status.find(NodeStatus::Active);
  // If the key does not exist or there are no active nodes, return -1
  if (it == nodes_by_status.end() || it->second.empty()) {
    return -1;
  }
  // If found, de-reference the iterator to get vector of active nodes' indices
  const std::vector<int>& activeNodeIndices = it->second;
  // Use a random number generator to create a uniform distribution over the range of active nodes
  std::uniform_int_distribution<> distrib(0, activeNodeIndices.size() - 1);
  // Select a random index from the vector of active node indices
  int randomNodeIndex = activeNodeIndices[distrib(gen)];
  // Return the index of the randomly selected active node
  return randomNodeIndex;
}

/**
 * @brief Executes the Activated Dijkstra algorithm on the graph to find
 *        the shortest path considering node status and range.
 *
 * @param bglGraph The BGLGraph object containing nodes, edges, and related information.
 * @param start Index of the starting node for the path.
 * @param end Index of the ending node for the path.
 * @param findToEndNode Boolean flag to specify whether to find a path to the end node.
 * @param debugDijkstra Whether to print debug information.
 * @return A tuple containing the destination node index, distance, and the path as a vector.
 */
std::tuple<int, double, std::vector<int>> ActivatedDijkstra( const BGLGraph& bglGraph, int start, int end, bool findToEndNode, bool debugDijkstra) {
  const Graph& graph = bglGraph.getGraph(); // Extracting Graph from BGLGraph
  int num_nodes = num_vertices(graph);
  std::vector<double> delta(num_nodes, std::numeric_limits<double>::infinity());
  std::vector<int> predecessor(num_nodes, -1);
  delta[start] = 0;

  using P = std::pair<double, int>; // Pair of (distance, node_index)
  std::priority_queue<P, std::vector<P>, std::greater<P>> min_heap;
  min_heap.emplace(0, start);

  if (debugDijkstra) {
    std::cout << "Initialized Dijkstra algorithm with " << num_nodes << " nodes.\n";
  }

  while (!min_heap.empty()) {
    double curr_dist = min_heap.top().first;
    int curr_node = min_heap.top().second;
    min_heap.pop();

    if (debugDijkstra) {
      std::cout << "Processing node " << curr_node << " with distance " << curr_dist << "\n";
    }

    // Check if we should find path to a specific 'end' node
    if (findToEndNode && curr_node == end) {
      std::vector<int> path;
      for (int at = curr_node; at != -1; at = predecessor[at]) {
        path.push_back(at);
      }
      std::reverse(path.begin(), path.end());
      return {curr_node, curr_dist, path};
    }
    // Original behavior: stop at any active node
    else if (!findToEndNode && graph[curr_node].status == NodeStatus::Active && curr_node != start) {
      std::vector<int> path;
      for (int at = curr_node; at != -1; at = predecessor[at]) {
        path.push_back(at);
      }
      std::reverse(path.begin(), path.end());
      if (debugDijkstra) {
        std::cout << "Found path to an active node " << curr_node << ".\n";
      }
      return {curr_node, curr_dist, path};
    }

    boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = out_edges(curr_node, graph); ei != ei_end; ++ei) {
      int neighbor = target(*ei, graph);
      double weight = get(boost::edge_weight_t(), graph, *ei);

      if (debugDijkstra) {
        std::cout << "Checking edge to neighbor " << neighbor << " with weight " << weight << "\n";
      }

      if (curr_dist + weight < delta[neighbor]) {
        delta[neighbor] = curr_dist + weight;
        predecessor[neighbor] = curr_node;
        min_heap.emplace(delta[neighbor], neighbor);

        if (debugDijkstra) {
          std::cout << "Updating distance for neighbor " << neighbor << " to " << delta[neighbor] << "\n";
        }
      }
    }
  }

  if (debugDijkstra) {
    std::cout << "Dijkstra algorithm completed.\n";
  }
  return {-1, std::numeric_limits<double>::infinity(), {}};
}

/**
 * @brief Finds the local minimum based on distance and range between active nodes.
 *
 * @param bglGraph The BGL graph containing nodes and edges.
 * @param gen Random number generator.
 * @return A tuple containing the node index, range or distance, path if applicable, and a boolean flag indicating if the value is a range.
 */
std::tuple<int, double, std::vector<int>, bool> findLocalMinimum(const BGLGraph& bglGraph, std::mt19937& gen, bool debug, bool debugDijkstra) {

  const Graph& graph = bglGraph.getGraph(); // Extracting Graph from BGLGraph

  int startNode = chooseRandomActiveStartNode(bglGraph, gen);
  if (debug) std::cout << "Start Node: " << startNode << " with range: " << graph[startNode].range << "\n";

  auto result1 = ActivatedDijkstra(bglGraph, startNode, -1, false, debugDijkstra);

  int nextNode = std::get<0>(result1);

  double distanceToNextNode = std::get<1>(result1);

  if (debug) std::cout << "Next Node: " << nextNode << " with range: " << graph[nextNode].range << "\n";
  if (debug) std::cout << "Distance to next node: " << distanceToNextNode << "\n";

  double localMin = std::min({graph[startNode].range, distanceToNextNode});

  if (debug) std::cout << "Minimum between start node and distance to next node: " << localMin << "\n";

  if (localMin == graph[startNode].range) {
    if (debug) std::cout << "Done finding Local minimum\n ---EXIT 1--- \n";
    return {startNode, graph[startNode].range, {}, true};
  }

  while (true) {

    auto result2 = ActivatedDijkstra(bglGraph, nextNode, -1, false, debugDijkstra);

    int nextToNextNode = std::get<0>(result2);

    double distanceToNextToNextNode = std::get<1>(result2);

    if (debug) std::cout << "Next to Next Node: " << nextToNextNode << " with range: " << graph[nextToNextNode].range << "\n";
    if (debug) std::cout << "Distance to next to next node: " << distanceToNextToNextNode << "\n";

    localMin = std::min({graph[nextNode].range, distanceToNextNode, graph[nextToNextNode].range, distanceToNextToNextNode});

    if (debug) std::cout << "Minimum between next node, distance to next node, next to next node, distance to next to next node: " << localMin << "\n";

    if (localMin == graph[nextNode].range) {
      if (debug) std::cout << "Done finding Local minimum\n ---EXIT 2--- \n";
      return {nextNode, graph[nextNode].range, {}, true};
    }

    else if (localMin == distanceToNextNode) {
      if (debug) std::cout << "Done finding Local minimum\n ---EXIT 3--- \n";
      return {nextNode, distanceToNextNode, std::get<2>(result1), false};
    }

    //else if (localMin == graph[nextToNextNode].range) {
    //  if (debug) std::cout << "Done finding Local minimum\n ---EXIT 4--- \n";
    //  return {nextToNextNode, graph[nextToNextNode].range, {}, true};
    //}

    //else if (localMin == distanceToNextToNextNode) {
    else {
      nextNode = nextToNextNode;
      distanceToNextNode = distanceToNextToNextNode;
      std::get<2>(result1) = std::get<2>(result2);

      if (debug) std::cout << "One loop in while without EXIT \n";
      if (debug) std::cout << "Next Node: " << nextNode << " with range: " << graph[nextNode].range << "\n";
      if (debug) std::cout << "Distance to next node: " << distanceToNextNode << "\n";
    }
  }
}
