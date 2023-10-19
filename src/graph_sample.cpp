#include "graph.hpp"

std::unordered_map<std::string, int> calculateEntanglementEntropy( const BGLGraph& bglGraph, int num_subsystems) {

  // Map to store the counts of each configuration of subsystems within clusters
  std::unordered_map<std::string, int> C;

  // Access the map of clusters from the BGLGraph object
  const auto& nodesByClusterIndex = bglGraph.getNodesByClusterIndex();

  // Get the underlying graph
  const auto& graph = bglGraph.getGraph();

  // Iterate through each cluster
  for (const auto& [clusterIndex, nodes] : nodesByClusterIndex) {

    // If the cluster has no nodes, skip this iteration
    if (nodes.empty()) {
      continue;
    }

    // Vector to represent the presence of each subsystem in the cluster
    std::vector<int> subsystemPresence(num_subsystems + 1, 0);

    // Iterate through each node in the cluster to collect subsystem indices
    for (const auto& nodeIndex : nodes) {
      subsystemPresence[graph[nodeIndex].subsystem] = 1;
    }

    // Convert subsystemPresence to string to use it as a key in map C
    std::string key;
    for (const auto& presence : subsystemPresence) {
      key += std::to_string(presence) + ',';
    }
    key.pop_back();  // Remove trailing comma

    // Increment count for this configuration in map C
    C[key]++;
  }

  return C;
}
