//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp" // Assuming this includes the BGLGraph definition

std::set<int> getUniqueClusters(const BGLGraph& bglGraph) {
  const Graph& g = bglGraph.getGraph();
  std::set<int> uniqueClusters;
  for (auto vp = vertices(g); vp.first != vp.second; ++vp.first) {
    VertexDescriptor v = *vp.first;
    const Node& node = g[v]; // Accessing the node directly from the graph
    uniqueClusters.insert(node.clusterIndex);
  }
  return uniqueClusters;
}

// Function to sample nodes until all clusters are represented at least once
int sampleNodes(const BGLGraph& bglGraph, std::mt19937& gen) {
  const Graph& g = bglGraph.getGraph();
  std::set<int> uniqueClusters = getUniqueClusters(bglGraph);

  // Debug statement to print the number of unique clusters
  //std::cout << "Number of unique clusters: " << uniqueClusters.size() << std::endl;

  std::set<int> sampledClusters;
  int samples = 0;
  std::vector<VertexDescriptor> nodes(vertices(g).first, vertices(g).second);

  while (sampledClusters.size() < uniqueClusters.size()) {
    std::shuffle(nodes.begin(), nodes.end(), gen);
    for (const VertexDescriptor& v : nodes) {
      const Node& node = g[v];
      // if (sampledClusters.insert(node.clusterIndex).second) {
      //   samples++; // Increment samples only if the cluster hasn't been sampled before
      // }
      sampledClusters.insert(node.clusterIndex);
      samples++;
      // Check if all clusters have been sampled after each node is sampled
      if (sampledClusters.size() == uniqueClusters.size()) {
        break; // Terminate the loop the moment the last unique cluster is sampled
      }
    }
    if (sampledClusters.size() == uniqueClusters.size()) {
      break; // Ensure the outer loop also terminates
    }
  }
  return samples;
  // samples returns the number of measurments made
}

// Function to compute average samples over multiple trials
double computeAverageSamples(const BGLGraph& bglGraph, int trials, std::mt19937& gen) {
  int totalSamples = 0;
  for (int i = 0; i < trials; ++i) {
    totalSamples += sampleNodes(bglGraph, gen);
  }
  return static_cast<double>(totalSamples) / trials;
}
