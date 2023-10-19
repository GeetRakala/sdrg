#include "graph.hpp"

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
