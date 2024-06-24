#include "graph.hpp" // Assuming this includes the BGLGraph definition

int chooseRandomNode(const BGLGraph& bglGraph, std::mt19937& gen) {
  std::uniform_int_distribution<> distrib(0, num_vertices(bglGraph.getGraph()) - 1);
  int randomNode = distrib(gen);
  std::cout << "Chosen random node: " << randomNode << std::endl;
  return randomNode;
}

int getClusterByNode(const BGLGraph& bglGraph, int nodeIndex) {
  auto vertexIt = vertices(bglGraph.getGraph());
  int clusterIndex = bglGraph.getGraph()[*(vertexIt.first + nodeIndex)].clusterIndex;
  std::cout << "Node " << nodeIndex << " is in cluster " << clusterIndex << std::endl;
  return clusterIndex;
}

double averageSpinsForAllClusters(const BGLGraph& bglGraph, std::mt19937& gen) {
  const int trials = 10; // Number of experiments
  int totalSpins = 0;
  int N = num_vertices(bglGraph.getGraph()); // Total number of vertices in the graph
  int M = bglGraph.getNodesByClusterIndex().size(); // Total number of clusters

  std::cout << "Starting trials. Total vertices (N): " << N << ", Total clusters (M): " << M << std::endl;

  for (int trial = 0; trial < trials; ++trial) {
    std::cout << "Trial number" << trial << std::endl;
    getchar();

    std::set<int> sampledClusters; // To keep track of sampled clusters
    int spinsCount = 0;

    while (sampledClusters.size() < static_cast<size_t>(M)) {
      int node = chooseRandomNode(bglGraph, gen); // Randomly pick a node
      int cluster = getClusterByNode(bglGraph, node); // Get cluster of the picked node

      // If the cluster is not already in the set, add it and increment the count
      if (sampledClusters.find(cluster) == sampledClusters.end()) {
        sampledClusters.insert(cluster);
        spinsCount++;
      }
    }

    totalSpins += spinsCount;
    std::cout << "Trial " << trial + 1 << ": Total spins so far: " << totalSpins << std::endl;
  }

  double average = static_cast<double>(totalSpins) / trials;
  std::cout << "Average spins for all clusters: " << average << std::endl;

  return average;
}
