#include "graph.hpp"

std::pair<int, std::unordered_map<int, int>> calculateClusterStatistics(const BGLGraph& bglGraph, int num_threads) {
    const auto& nodesByClusterIndex = bglGraph.getNodesByClusterIndex();
    std::unordered_map<int, int> clusterSizeCount;
    std::mutex mutex;
    std::vector<std::thread> threads;
    int totalClusters = 0; // Count of non-zero clusters

    // Function to process a subset of clusters
    auto processClusterSubset = [&](auto itBegin, auto itEnd) {
        std::unordered_map<int, int> localClusterSizeCount;
        int localTotalClusters = 0;

        for (auto it = itBegin; it != itEnd; ++it) {
            int clusterSize = it->second.size();
            if (clusterSize > 0) {
                localTotalClusters++;
                localClusterSizeCount[clusterSize]++;
            }
        }

        // Merge local map and total cluster count into global map in a thread-safe manner
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& pair : localClusterSizeCount) {
            clusterSizeCount[pair.first] += pair.second;
        }
        totalClusters += localTotalClusters;
    };

    // Create threads
    auto it = nodesByClusterIndex.begin();
    auto end = nodesByClusterIndex.end();
    int chunkSize = nodesByClusterIndex.size() / num_threads + (nodesByClusterIndex.size() % num_threads != 0);

    for (int i = 0; i < num_threads; ++i) {
        auto nextIt = std::next(it, std::min(chunkSize, static_cast<int>(std::distance(it, end))));
        threads.emplace_back(processClusterSubset, it, nextIt);
        it = nextIt;
        if (it == end) {
            break;
        }
    }

    // Join threads
    for (auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    return {totalClusters, clusterSizeCount};
}

std::unordered_map<std::string, int> calculateEntanglementCounts(const BGLGraph& bglGraph, int num_subsystems, int num_threads) {
  std::unordered_map<std::string, int> C;
  const auto& nodesByClusterIndex = bglGraph.getNodesByClusterIndex();
  const auto& graph = bglGraph.getGraph();
  std::mutex mutex;

  // Function to process a single cluster
  auto processCluster = [&C, &graph, &mutex, num_subsystems](const auto& clusterPair) {
    const auto& [clusterIndex, nodes] = clusterPair;
    if (nodes.empty()) return;

    boost::dynamic_bitset<> subsystemPresence(num_subsystems+1);
    for (const auto& nodeIndex : nodes) {
      subsystemPresence.set(graph[nodeIndex].subsystem);
    }

    std::stringstream ss;
    ss << subsystemPresence;
    std::string key = ss.str();

    {
      std::lock_guard<std::mutex> lock(mutex);
      C[key]++;
    }
  };

  // Create threads and manage them: only max_threads concurrently
  std::vector<std::thread> threads;
  for (const auto& clusterPair : nodesByClusterIndex) {
    if (threads.size() >= num_threads) {
      // Wait for one thread to finish
      threads.front().join();
      threads.erase(threads.begin());
    }
    threads.emplace_back(processCluster, clusterPair);
  }

  // Join remaining threads
  for (auto& t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }

  return C;
}


int calculateVonNeumannEntropy(const std::unordered_map<std::string, int>& entanglementCounts, int subsystems) {

    boost::dynamic_bitset<> vonNeumannKey(subsystems + 1);
    vonNeumannKey.set(0);
    vonNeumannKey.set(1);
    std::stringstream vonNeumannStr;
    vonNeumannStr << vonNeumannKey;
    std::string vonNeumannKeyStr = vonNeumannStr.str();

    try {
        return entanglementCounts.at(vonNeumannKeyStr);
    } catch (const std::out_of_range&) {
        std::cout << "Warning: Required configurations for Entanglement Entropy not found.\n";
        return 0;
    }
}

std::tuple<int, int> calculateMutualInfo(const std::unordered_map<std::string, int>& entanglementCounts, int subsystems) {
    boost::dynamic_bitset<> entanglementNegativityKey(subsystems + 1);
    entanglementNegativityKey.set(0);
    entanglementNegativityKey.set(2);
    std::stringstream entanglementNegativityStr;
    entanglementNegativityStr << entanglementNegativityKey;
    std::string entanglementNegativityKeyStr = entanglementNegativityStr.str();

    int entNegativityCount = 0;
    auto entNegativityIt = entanglementCounts.find(entanglementNegativityKeyStr);
    if (entNegativityIt != entanglementCounts.end()) {
        entNegativityCount = entNegativityIt->second;
    }

    boost::dynamic_bitset<> highVonNeumannKey(subsystems + 1);
    highVonNeumannKey.set(0);
    highVonNeumannKey.set(1);
    highVonNeumannKey.set(2);
    std::stringstream highVonNeumannStr;
    highVonNeumannStr << highVonNeumannKey;
    std::string highVonNeumannKeyStr = highVonNeumannStr.str();

    int highVonNeumannCount = 0;
    auto highVonNeumannIt = entanglementCounts.find(highVonNeumannKeyStr);
    if (highVonNeumannIt != entanglementCounts.end()) {
        highVonNeumannCount = highVonNeumannIt->second;
    }

    int mutualInformation = 2 * entNegativityCount + highVonNeumannCount;
    return std::make_tuple(mutualInformation, entNegativityCount);
}
