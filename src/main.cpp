//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================
#include "graph.hpp"

int main() {
  ConfigParams params;

  // read the parameter values from the config.txt file
  if (!parseConfig("config.txt", params)) {
    return 1;  // Exit if parsing failed
  }

  // intialise random number sequence
  std::mt19937 gen(params.seed);

  // create a set of nodes that make the lattice and add edges between the nodes.
  BGLGraph bglGraph = createLatticeGraph(params.latticeSize, params.delta, params.graphType, params.distType, gen);

  // assign subsystem indices to the nodes
  if (params.partitionType == "bipartite") {
    initializeBipartiteChain(bglGraph);
    params.subsystems = 1;
  }
  else if (params.partitionType == "random") {
    initializeSubsystem(bglGraph, params.pee, params.subsystems, gen);
  }

  // create a JSON image of the graph
  if (params.json) generateDataFile(bglGraph, "graph_data");

  // initialize the graph snapshot array
  // This will be used as an array only if history is on. Otherwise it will
  // be used to store only the snapshot with the largest minima value
  std::vector<IterationSnapshot> snapshots;
  double largestLocalMinimaValue = -std::numeric_limits<double>::infinity();
  IterationSnapshot snapshot(0, std::make_tuple(0, largestLocalMinimaValue, std::vector<int>(), false));
  double maxMinima = -std::log(params.temp);

  for (int j = 0; j < num_vertices(bglGraph.getGraph()); ++j) {
    if (params.debugMain) std::cout << "Iteration : " << j << '\n';

    std::tuple<int, double, std::vector<int>, bool> result;

    // params.dumb is the switch which chooses between dumb sdrg decimations vs Kovcs-Igloi
    if (params.dumb) {
      result = findMinimumEdgeOrNodeRange(bglGraph);
    } else {
      result = findLocalMinimum(bglGraph, gen, params.debugFlm, params.debugDijkstra);
    }

    double currentLocalMinimaValue = std::get<1>(result);

    if (currentLocalMinimaValue > maxMinima) {
      if (params.dumb) break;
      continue;
    }

    if (!params.history && currentLocalMinimaValue > largestLocalMinimaValue) {
      largestLocalMinimaValue = currentLocalMinimaValue;
      snapshot = IterationSnapshot(j, result);
    }

    if (params.history) {
      snapshot = IterationSnapshot(j, result);
      snapshots.push_back(snapshot);
    }

    if (params.debugMain) {
      const auto& [node, value, path, isRange] = result;
      std::cout << "Node: " << node << '\n';
      std::cout << (isRange ? "Range: " : "Distance: ") << value << '\n';
      if (!path.empty()) {
        std::cout << "Path: ";
        for (const auto& p : path) {
          std::cout << p << " -> ";
        }
        std::cout << "End\n";
      }
      std::cout << "---------------------------------------\n";
    }

    if (params.dumb) {
      do_dumb_decimate(bglGraph, result);
    } else {
      do_decimate(bglGraph, result, params.debugDecimate, params.debugRemoveSelfLoops, params.debugReassignEdges, params.debugRemoveDuplicateEdges, params.debugUpdateAllEdgeDistancesFromNode, params.debugProcessNegativeEdge);
    }

    if (params.debugMain) std::cout << "---------------------------------------\n";
    if (params.json) generateDataFile(bglGraph, "graph_data");
  }

  if (params.history) {
    std::sort(snapshots.begin(), snapshots.end(), [](const IterationSnapshot& a, const IterationSnapshot& b) {
        return std::get<1>(a.localMinimaResult) < std::get<1>(b.localMinimaResult);
        });

    // Extract and display details from snapshots
    const auto& localMinimaDetails = snapshots.back().localMinimaResult;
    int node = std::get<0>(localMinimaDetails);
    double value = std::get<1>(localMinimaDetails);
    bool isRange = std::get<3>(localMinimaDetails);
    std::cout << "Max Minima: " << maxMinima << '\n';
    std::cout << "Details of the largest local minimum:\n";
    std::cout << "Iteration Index: " << snapshot.decimationIndex << '\n';
    std::cout << "Node: " << node << '\n';
    std::cout << (isRange ? "Range: " : "Distance: ") << value << '\n';
    std::cout << "---------------------------------------\n";

    for (const auto& snap : snapshots) {
      const auto& details = snap.localMinimaResult;
      std::cout << "Iteration Index: " << snap.decimationIndex
        << ", Node Index: " << std::get<0>(details)
        << ", Local Minimum: " << std::get<1>(details) << '\n';
    }
  }

  // entropy calculations
  std::string directoryName = "csvfiles";

  // Check if the directory exists and create it if it doesn't
  if (!std::filesystem::exists(directoryName)) {
    std::filesystem::create_directory(directoryName);
  }

  std::string baseFilename = generateFilename();

  std::ofstream outFileClusters(directoryName + "/" + baseFilename + "_cluster_statistics.csv");
  outFileClusters << "Cluster Size, Cluster Count\n";

  std::ofstream outFileResults(directoryName + "/" + baseFilename + "_results.csv");

  if (params.subsystems == 1) {
    outFileResults << "TotalClusters, EntEntropy\n";
  }
  else if (params.subsystems == 2) {
    outFileResults << "TotalClusters, MutualInfo, EntNegativity\n";
  }

  auto [totalClusters, clusterSizeCount] = calculateClusterStatistics(bglGraph, params.threads);
  if (params.verbose) std::cout << "Total number of clusters: " << totalClusters << "\n";
  for (const auto& [size, count] : clusterSizeCount) {
    if (params.verbose) std::cout << "Number of clusters of size " << size << " : " << count << "\n";
    outFileClusters << size << ", " << count << "\n";
  }

  auto entanglementCounts = calculateEntanglementCounts(bglGraph, params.subsystems, params.threads);

  if (params.verbose) {
    for (const auto& [config, count] : entanglementCounts) {
      std::cout << "Configuration " << config << " has a count of: " << count << "\n";
    }
  }

  if (params.subsystems == 1) {
    int entanglementEntropy = calculateVonNeumannEntropy(entanglementCounts, params.subsystems);
    if (params.verbose) std::cout << "Entanglement Entropy = " << entanglementEntropy << "\n";
    outFileResults << totalClusters << "," << entanglementEntropy << "\n";
  }

  if (params.subsystems == 2) {
    auto [mutualInformation, entanglementNegativity] = calculateMutualInfo(entanglementCounts, params.subsystems);
    if (params.verbose) {
      std::cout << "Mutual Information = " << mutualInformation << "\n";
      std::cout << "Entanglement Negativity = " << entanglementNegativity << "\n";
    }
    outFileResults << totalClusters << "," << mutualInformation << "," << entanglementNegativity << "\n";
  }

  outFileResults.close();
  outFileClusters.close();

  logMetadata(params, baseFilename, directoryName);

  std::cout << "persistence trials: " << params.persTrials << std::endl;
  double averageSamples = computeAverageSamples(bglGraph, params.persTrials, gen);

  std::cout << "Average number of nodes to be sampled to cover all clusters at least once: "
    << averageSamples << std::endl;

  return 0;
}
