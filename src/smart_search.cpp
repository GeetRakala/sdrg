//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"

int chooseRandomActiveStartNode(const BGLGraph& graph, std::mt19937& gen) {
  auto nodes_by_status = graph.getNodesByStatus();
  auto it = nodes_by_status.find(NodeStatus::Active);
  if (it == nodes_by_status.end() || it->second.empty()) {
    return -1;
  }
  const std::vector<int>& activeNodeIndices = it->second;
  std::uniform_int_distribution<> distrib(0, activeNodeIndices.size() - 1);
  int randomNodeIndex = activeNodeIndices[distrib(gen)];
  return randomNodeIndex;
}

std::vector<int> constructPath(int node, const std::vector<int>& predecessors) {
  std::vector<int> path;
  for (; node != -1; node = predecessors[node]) {
    path.push_back(node);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

// The activated Dijkstra algorithm
// using Fibonacci heap data structure
// TODO: find all distances from all active sites simultaneously.
// TODO: Delete inactive sites that have already been crossed during the search.

std::tuple<int, double, std::vector<int>> ActivatedDijkstra(
    const BGLGraph& bglGraph,
    int start,
    int end,
    bool findToEndNode,
    bool debugDijkstra
    ) {
  const Graph& graph = bglGraph.getGraph();
  std::vector<double> distances(num_vertices(graph), std::numeric_limits<double>::infinity());
  std::vector<int> predecessors(num_vertices(graph), -1);

  if (start < 0 || start >= num_vertices(graph)) {
    std::cout << "Start : " << start << "\n";
    throw std::out_of_range("Start index out of range");
  }

  distances[start] = 0;

  using P = std::pair<double, int>;
  std::priority_queue<P, std::vector<P>, std::greater<P>> min_heap;
  min_heap.emplace(0, start);

  while (!min_heap.empty()) {
    double curr_dist = min_heap.top().first;
    int curr_node = min_heap.top().second;
    min_heap.pop();

    // Maybe add a condition here to check if an inactive node has already been visited.
    // If yes, skip the already visited node. Need to check an example of where this works.
    // If current distance is greater than the shortest known distance for this node, skip.
    // It is also possible that I can avoid a lot of complexity of reassignemt if I naturally
    // skip nodes with distances outside the range/2 (or some such thing) of the origin node.

    if (curr_dist > distances[curr_node]) continue;

    if (graph[curr_node].status == NodeStatus::Active &&
        (!findToEndNode || curr_node == end) &&
        curr_node != start) {
      return {curr_node, curr_dist, constructPath(curr_node, predecessors)};
    }

    for (auto [ei, ei_end] = out_edges(curr_node, graph); ei != ei_end; ++ei) {
      int neighbor = target(*ei, graph);
      double weight = get(boost::edge_weight_t(), graph, *ei);

      if (findToEndNode && graph[neighbor].status == NodeStatus::Active && neighbor != end) {
        continue;
      }

      if (curr_dist + weight < distances[neighbor]) {
        distances[neighbor] = curr_dist + weight;
        predecessors[neighbor] = curr_node;
        min_heap.emplace(distances[neighbor], neighbor);
      }
    }
  }
  return {-1, std::numeric_limits<double>::infinity(), {}};
}


// Finds the local minimum based on distance and range between active nodes.
// TODO: Possible optimisation possible here? Have to see.
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
