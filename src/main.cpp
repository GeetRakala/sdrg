//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================
#include "graph.hpp"

int main() {
  ConfigParams params;

  if (!parseConfig("config.txt", params)) {
    return 1;  // Exit if parsing failed
  }

  std::mt19937 gen(params.seed);

  BGLGraph bglGraph = createSquareLatticeGraph(params.latticeSize, params.theta, gen);
  initializeSubsystem(bglGraph, params.pee, params.subsystems, gen);

  if (params.json) generateDataFile(bglGraph, "graph_data");

  std::vector<IterationSnapshot> snapshots;
  double largestLocalMinimaValue = -std::numeric_limits<double>::infinity();
  IterationSnapshot snapshot(0, std::make_tuple(0, largestLocalMinimaValue, std::vector<int>(), false));
  double maxMinima = -std::log(params.temp);

  for (int j = 0; j < params.latticeSize * params.latticeSize; ++j) {
    if (params.debugMain) std::cout << "Iteration : " << j << '\n';

    std::tuple<int, double, std::vector<int>, bool> result;

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

  // Calculate and display entanglement entropy details
  auto entanglementCounts = calculateEntanglementEntropy(bglGraph, params.subsystems);

  for (const auto& [config, count] : entanglementCounts) {
    std::cout << "Configuration " << config << " has a count of: " << count << "\n";
  }

  // Calculate vonNeuman entanglement entropy
  if (params.subsystems == 1) {
    int entanglementEntropy = 0;
    try {
      entanglementEntropy = entanglementCounts.at("1,1");
    } catch (const std::out_of_range&) {
      std::cout << "Warning: Required configurations for Entanglement Entropy not found.\n";
    }
    std::cout << "Entanglement Entropy = " << entanglementEntropy << "\n";
  }

  // Calculate Mutual Information and Entanglement Negativity when params.subsystems is 2
  if (params.subsystems == 2) {
    int mutualInformation = 0;
    int entanglementNegativity = 0;

    try {
      mutualInformation = 2 * entanglementCounts.at("1,0,1") + entanglementCounts.at("1,1,1");
      entanglementNegativity = entanglementCounts.at("1,0,1");
    } catch (const std::out_of_range&) {
      std::cout << "Warning: Required configurations for Mutual Information or Entanglement Negativity not found.\n";
    }

    std::cout << "Mutual Information = " << mutualInformation << "\n";
    std::cout << "Entanglement Negativity = " << entanglementNegativity << "\n";
  }


  return 0;
}
