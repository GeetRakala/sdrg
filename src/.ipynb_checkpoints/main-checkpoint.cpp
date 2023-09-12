//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"

struct IterationSnapshot {
  int decimationIndex;
  std::tuple<int, double, std::vector<int>, bool> localMinimaResult;
  //BGLGraph graphState;

  // Add constructor
  //IterationSnapshot(int decimationIndex, std::tuple<int, double, std::vector<int>, bool> localMinimaResult, BGLGraph graphState)
   // : decimationIndex(decimationIndex),
    //localMinimaResult(localMinimaResult),
    //graphState(graphState) {}
//};
  IterationSnapshot(int decimationIndex, std::tuple<int, double, std::vector<int>, bool> localMinimaResult)
    : decimationIndex(decimationIndex),
    localMinimaResult(localMinimaResult) {}
};

int main() {

  // Parse the configuration file to retrieve constants
  ConfigParams params;
  if (parseConfig("config.txt", params)) {
  }

  // Initialize the random number generator with the given seed
  std::mt19937 gen(params.seed);

  // Loop of N trials
  for (int i = 0; i < 1; ++i) {
    // Create a square lattice graph with specified size
    BGLGraph bglGraph = createSquareLatticeGraph(params.latticeSize,params.theta,gen);

    // Generate a JSON file containing graph data
    if (params.json) generateDataFile(bglGraph, "graph_data");

    // Initialize snapshots vector
    std::vector<IterationSnapshot> snapshots;

    // Initialize the localMinimaValue to a really low value
    double largestLocalMinimaValue = -std::numeric_limits<double>::infinity();

    //IterationSnapshot snapshot(0, std::make_tuple(0, largestLocalMinimaValue, std::vector<int>(), false), bglGraph);
    IterationSnapshot snapshot(0, std::make_tuple(0, largestLocalMinimaValue, std::vector<int>(), false));

    for (int j = 0; j < params.latticeSize*params.latticeSize; ++j) {
      if (params.debugMain) std::cout << "Iteration : " << j << '\n';

      std::tuple<int, double, std::vector<int>, bool> result;
      // Find local minima
      if (params.dumb) result = findMinimumEdgeOrNodeRange(bglGraph);
      if (!params.dumb) result = findLocalMinimum(bglGraph, gen, params.debugFlm, params.debugDijkstra);

      if (!params.history) {
        // Get the localMinimaValue from the result
        double currentLocalMinimaValue = std::get<1>(result);
        // If the new snapshot has a larger local minima, replace the current snapshot
        if (currentLocalMinimaValue > largestLocalMinimaValue) {
          largestLocalMinimaValue = currentLocalMinimaValue;
          // Assign a new snapshot
          //snapshot = IterationSnapshot(j, result, bglGraph);
          snapshot = IterationSnapshot(j, result);
        }
      }

      if (params.history) {
        //snapshot = IterationSnapshot(j, result, bglGraph);
        snapshot = IterationSnapshot(j, result);
        snapshots.push_back(snapshot);
      }

      if (params.debugMain) {
        int node = std::get<0>(result);
        double value = std::get<1>(result);
        const auto& path = std::get<2>(result);
        bool isRange = std::get<3>(result);
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

      if (!params.dumb) do_decimate(bglGraph, result, params.debugDecimate, params.debugRemoveSelfLoops, params.debugReassignEdges, params.debugRemoveDuplicateEdges, params.debugUpdateAllEdgeDistancesFromNode, params.debugProcessNegativeEdge);
      if (params.dumb) do_dumb_decimate(bglGraph, result);

      if (params.debugMain) {
        std::cout << "---------------------------------------\n";
      }

      if (params.json) generateDataFile(bglGraph, "graph_data");
    } //end of decimate iteration

    if (params.history) {
      // Sorting snapshots based on the localMinimaValue in ascending order
      std::sort(snapshots.begin(), snapshots.end(), [](const IterationSnapshot& a, const IterationSnapshot& b) {
          return std::get<1>(a.localMinimaResult) < std::get<1>(b.localMinimaResult);
          });
    }

    // Declare a tuple to hold localMinimaDetails
    std::tuple<int, double, std::vector<int>, bool> localMinimaDetails;

    // Extract and print details of largest local minimum
    if (params.history) localMinimaDetails = snapshots.back().localMinimaResult;
    if (!params.history) localMinimaDetails = snapshot.localMinimaResult;
    int node = std::get<0>(localMinimaDetails);
    double value = std::get<1>(localMinimaDetails);
    bool isRange = std::get<3>(localMinimaDetails);
    std::cout << "Details of the largest local minimum:\n";
    std::cout << "Iteration Index: " << snapshot.decimationIndex << '\n';
    std::cout << "Node: " << node << '\n';
    std::cout << (isRange ? "Range: " : "Distance: ") << value << '\n';
    std::cout << "---------------------------------------\n";

    if (params.history) {
      // Print all iteration indices and local minima in ascending order
      std::cout << "\nFull list of iteration indices and local minima:\n";
      for (const auto& snapshot : snapshots) {
        auto& localMinimaDetails = snapshot.localMinimaResult;
        int iterationIndex = snapshot.decimationIndex;
        int nodeIndex = std::get<0>(localMinimaDetails);
        double localMinimaValue = std::get<1>(localMinimaDetails);

        std::cout << "Iteration Index: " << iterationIndex
          << ", Node Index: " << nodeIndex
          << ", Local Minimum: " << localMinimaValue << '\n';
      }
      std::cout << "---------------------------------------\n";
    }

  } //N trials for loop
  return 0;
}
