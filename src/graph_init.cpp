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
void initializeRangeAndDistance(BGLGraph& graph, double delta, std::string& latticeType, std::string& distType, std::mt19937& gen) {

  double h_max;
  std::uniform_real_distribution<> dis_h;

  // Calculate h_max only if distType is "box"
  if (distType == "box") {
    if (latticeType == "square") {
      h_max = std::exp(1.6784 + delta);
    } else if (latticeType == "chain") {
      h_max = std::exp(delta);
    }
    dis_h = std::uniform_real_distribution<>(0.0, h_max);
  }
  else if (distType == "fixed") {
    if (latticeType == "square") {
      h_max = std::exp(-0.17034 + delta);
    } else if (latticeType == "chain") {
      h_max = std::exp(delta);
    }
  }
  //std::cout << "h_max = " << h_max << "\n";
  // Define distribution for J
  std::uniform_real_distribution<> dis_J(0.0, 1.0);  // Uniform distribution between 0 and 1

  // Update the range for each node
  for (int i = 0; i < num_vertices(graph.getGraph()); ++i) {
    double h;
    if (distType == "box") {
      h = dis_h(gen);  // Draw h for box distribution
    } else if (distType == "fixed") {
      h = h_max; // Fixed h
    }
    double range;
    if (distType == "box") {
      range = -std::log(h);
    } else if (distType == "fixed") {
      range = -std::log(h);
    }
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

BGLGraph createLatticeGraph(int L, double delta, std::string& latticeType, std::string& distType, std::mt19937& gen) {

  if (latticeType == "square") {
    int num_nodes = L * L;
    BGLGraph graph(num_nodes);

    for (int i = 0; i < L; ++i) {
      for (int j = 0; j < L; ++j) {
        int index = i * L + j;
        graph.addNode(index, 0.0, NodeStatus::Active, index, 0);

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
    initializeRangeAndDistance(graph, delta, latticeType, distType, gen);
    return graph;
  }
  else if (latticeType == "chain") {
    int num_nodes = L;
    BGLGraph graph(num_nodes);
    for (int i = 0; i < L; ++i) {
      graph.addNode(i, 0.0, NodeStatus::Active, i, 0);
      if (i + 1 < L) {
        graph.addEdge(i, i + 1, 0.0);
      }
    }
    initializeRangeAndDistance(graph, delta, latticeType, distType, gen);
    return graph;
  }
  else {
    // Handle unknown graph type
    throw std::invalid_argument("Unknown lattice type: " + latticeType);
  }

}

std::vector<double> generateSubsystemProbabilities(double pee, int num_subsystems) {
  // Initialize the vector with default probabilities
  std::vector<double> subsystem_probs(num_subsystems+1, 0.0);

  // 0 is always rest of the system B
  subsystem_probs[0] = 1 - pee;
  // [1,num_subsystems] are the subsystems A1, A2, ... , AN
  for (int i = 1; i <= num_subsystems; ++i) subsystem_probs[i] = pee / num_subsystems;

  return subsystem_probs;
}


void initializeSubsystem(BGLGraph& graph, double pee, int num_subsystems, std::mt19937& gen) {

  std::vector<double> subsystem_probs = generateSubsystemProbabilities(pee, num_subsystems);

  // Create a distribution for subsystems based on the probabilities
  std::discrete_distribution<> subsystem_distrib(subsystem_probs.begin(), subsystem_probs.end());

  for (int i = 0; i < num_vertices(graph.getGraph()); ++i) {
    // Assign random subsystem to the node based on the defined probabilities
    int random_subsystem = subsystem_distrib(gen);
    graph.updateNodeSubsystem(i, random_subsystem);
  }
}

void initializeBipartiteChain(BGLGraph& graph) {
  const auto totalVertices = num_vertices(graph.getGraph());

  for (int i = 0; i < totalVertices; ++i) {
    int subsystem = (i < totalVertices / 2) ? 0 : 1;
    graph.updateNodeSubsystem(i, subsystem);
  }
}
