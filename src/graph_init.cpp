//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Initializes the activation status of nodes in a given Graph.
 * ...
 * @param graph Reference to the BGLGraph object whose nodes' statuses should be initialized.
 * @param gen Reference to a random number generator of type std::mt19937.
 */
void initializeNodeStatus(BGLGraph& graph, std::mt19937& gen) {
  std::uniform_int_distribution<> distrib(0, 2);

  for (int i = 0; i < num_vertices(graph.getGraph()); ++i) {
    int random_status = distrib(gen);

    switch (random_status) {
      case 0:
        graph.updateNodeStatus(i, NodeStatus::Dead);
        //graph.updateNodeStatus(i, NodeStatus::Active); //temp. change for testing
        break;
      case 1:
        graph.updateNodeStatus(i, NodeStatus::Active);
        break;
      case 2:
        graph.updateNodeStatus(i, NodeStatus::Inactive);
        break;
    }
  }
}

/**
 * @brief Initializes the range of nodes and distance of edges in a given Graph.
 * ...
 * @param graph Reference to the BGLGraph object whose nodes' ranges and edges' distances are to be initialized.
 * @param gen Reference to a random number generator of type std::mt19937.
 */
void initializeRangeAndDistance(BGLGraph& graph, double theta, std::mt19937& gen) {
  double h_max = std::exp(theta);  // Calculate h_max using theta
  std::uniform_real_distribution<> dis_h(0.0, h_max); // Uniform distribution between 0 and h_max
  std::uniform_real_distribution<> dis_J(0.0, 1.0);  // Uniform distribution between 0 and 1

  // Update the range for each node
  for (int i = 0; i < num_vertices(graph.getGraph()); ++i) {
    double h = dis_h(gen);  // Draw h and normalize it by dividing by h_max
    double range = -std::log(h);  // Natural logarithm transformation
    graph.updateNodeRange(i, range);
  }

  // Update the distance for each edge
  boost::graph_traits<Graph>::edge_iterator ei, eend;
  for (boost::tie(ei, eend) = edges(graph.getGraph()); ei != eend; ++ei) {
    double J = dis_J(gen);
    double distance = -std::log(J);  // Natural logarithm transformation
    graph.updateEdgeDistance(source(*ei, graph.getGraph()), target(*ei, graph.getGraph()), distance);
  }
}
/**
 * @brief Creates a square lattice graph of size L x L.
 * ...
 * @param L The length of the sides of the square lattice.
 * @param gen Reference to a random number generator of type std::mt19937.
 * @return The created BGLGraph object representing the square lattice.
 */
BGLGraph createSquareLatticeGraph(int L, double theta, std::mt19937& gen) {
  int num_nodes = L * L;
  BGLGraph graph(num_nodes);

  for (int i = 0; i < L; ++i) {
    for (int j = 0; j < L; ++j) {
      int index = i * L + j;
      graph.addNode(index, 0.0, NodeStatus::Active, index);

      if (j + 1 < L) {
        int right = i * L + j + 1;
        graph.addEdge(index, right, 0.0);
      }
      if (i + 1 < L) {
        int down = (i + 1) * L + j;
        graph.addEdge(index, down, 0.0);
      }
    }
  }
  initializeRangeAndDistance(graph, theta, gen);

  return graph;
}
