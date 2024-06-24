//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================
#pragma once
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/dynamic_bitset.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <sstream>
#include <random>
#include <queue>
#include <iostream>
#include <fstream>
#include <bitset>
#include <set>
#include <stdexcept>
#include <algorithm>
#include <execution>
#include <mutex>
#include <thread>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <unistd.h>


struct ConfigParams {
  int seed = 0;
  std::string graphType = "chain";
  std::string distType = "box";
  std::string partitionType = "bipartite";
  int latticeSize = 0;
  double delta = 0.0;
  double temp = 0.0;
  int trials = 0;
  int persTrials = 0;
  double pee = 0.0;
  int subsystems = 2;
  int threads = 10;
  bool history = false;
  bool json = false;
  bool dumb = true;
  bool verbose = false;
  bool debugMain = false;
  bool debugFlm = false;
  bool debugDijkstra = false;
  bool debugDecimate = false;
  bool debugRemoveSelfLoops = false;
  bool debugReassignEdges = false;
  bool debugRemoveDuplicateEdges = false;
  bool debugUpdateAllEdgeDistancesFromNode = false;
  bool debugProcessNegativeEdge = false;
};


struct IterationSnapshot {
  int decimationIndex;
  std::tuple<int, double, std::vector<int>, bool> localMinimaResult;
  IterationSnapshot(
      int decimationIndex,
      std::tuple<int, double, std::vector<int>, bool> localMinimaResult
      ) :
    decimationIndex(decimationIndex),
    localMinimaResult(localMinimaResult)
  {}
};


enum class NodeStatus { Dead, Active, Inactive };

struct Node {
  double range;           // Represents 'range' of the node.
  NodeStatus status;      // Status of the node.
  int index;              // Index of the node.
  int clusterIndex;       // Index of the cluster the node belongs to.
  int subsystem;          // Which subsystem does the node belong to.

  Node() : range(0), status(NodeStatus::Active), index(-1), clusterIndex(-1), subsystem(0) {}
  Node(double range, NodeStatus status, int index, int clusterIndex, int subsystem)
    : range(range), status(status), index(index), clusterIndex(clusterIndex), subsystem(subsystem) {}
};

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
        Node, EdgeWeightProperty> Graph;

typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;

typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;

/**
 * @brief A utility class to provide Graph functionalities like adding,
 * removing, updating nodes and edges using BGL.
 */
class BGLGraph {
  Graph g;
  std::unordered_map<NodeStatus, std::vector<int>> nodes_by_status;
  std::unordered_map<int, std::vector<int>> nodes_by_clusterIndex;
  std::unordered_map<bool, std::vector<int>> nodes_in_subsystem;

  public:
  // Constructor: Initializes Graph with a given size.
  BGLGraph(int size) : g(size) {}
  // Accessor method to get nodes by status
  const std::unordered_map<NodeStatus, std::vector<int>>& getNodesByStatus() const {
    return nodes_by_status;
  }
  // Accessor method to get nodes by cluster index
  const std::unordered_map<int, std::vector<int>>& getNodesByClusterIndex() const {
    return nodes_by_clusterIndex;
  }
  // Accessor method to get nodes by subsystem A
  const std::unordered_map<bool, std::vector<int>>& getNodesInSubsystem() const {
    return nodes_in_subsystem;
  }
  // Reference to the underlying graph
  // Non-const version
  Graph& getGraph() {
    return g;
  }
  // Const version
  const Graph& getGraph() const {
    return g;
  }

  // functions defined in graph_utilities.cpp
  void removeOutEdges(int nodeIndex);
  void removeInEdges(int nodeIndex);
  void addEdge(int from, int to, double distance);
  void removeEdge(int from, int to);
  void addNode(int index, double range, NodeStatus status, int clusterIndex, int subsystem);
  void updateNodeStatus(int node_index, NodeStatus new_status);
  void updateNodeRange(int node_index, double new_range);
  void updateNodeClusterIndex(int node_index, int new_clusterIndex);
  void updateNodeSubsystem(int node_index, int new_subsystem);
  void updateEdgeDistance(int from, int to, double new_distance);

  // functions defined in smart_utilities.cpp
  void updateAllEdgeDistancesFromNode(int node_index, bool debug, bool debugProcessNegativeEdge);
  void updateNeighborEdgeDistancesLastNode(int startNode, bool debug);
  void updateFirstNeighborEdgeDistancesLastNode(int lastNode, int firstNode, bool debug);
  void updateInactiveNeighborEdgeDistances(int sourceNode, int targetNode, bool debug);
  void removeDuplicateEdges(int sourceNode, bool debug);
  void reassignEdges(int newSourceNode, int oldSourceNode, bool debug);
  void reassignInactiveEdges(int newSourceNode, const std::vector<int>& path, bool debug);
  void removeSelfLoops(int sourceNode, bool debug);
  void processNegativeEdge(EdgeDescriptor edgeWithNegativeDistance, double negativeEdgeDistance, bool debug, std::vector<std::pair<EdgeDescriptor, double>>& newly_negative_edges);


  //functions defined in dumb_utilities.cpp
  void updateEdgesAndRemoveNode(int inputNode, bool debug);

};

// functions defined in graph_io.cpp
void generateDataFile(const BGLGraph& bglGraph, const std::string& filename);
void printGraphDetails(const BGLGraph& bglGraph, int L);
bool parseConfig(const std::string& fileName, ConfigParams& params);
std::string generateFilename();
void logMetadata(const ConfigParams& params, const std::string& baseFilename, const std::string& directoryName);

// functions defined in graph_init.cpp
void initializeNodeStatus(BGLGraph& graph, std::mt19937& gen);
void initializeRangeAndDistance(BGLGraph& graph, double delta, std::string& latticeType, std::string& distType, std::mt19937& gen);
BGLGraph createLatticeGraph(int L, double delta, std::string& latticeType, std::string& distType, std::mt19937& gen);
std::vector<double> generateSubsystemProbabilities(double p, int num_subsystems);
void initializeSubsystem(BGLGraph& graph, double p, int num_subsystems, std::mt19937& gen);
void initializeBipartiteChain(BGLGraph& graph);

// functions defined in smart_search.cpp
int chooseRandomActiveStartNode(const BGLGraph& graph, std::mt19937& gen);
std::tuple<int, double, std::vector<int>> ActivatedDijkstra( const BGLGraph& bglGraph, int start, int end, bool findToEndNode, bool debugDijkstra);
std::tuple<int, double, std::vector<int>, bool> findLocalMinimum(const BGLGraph& bglGraph, std::mt19937& gen, bool debug = false, bool debugDijkstra = false);

// functions defined in smart_decimate.cpp
void do_decimate(BGLGraph& bglGraph, const std::tuple<int, double, std::vector<int>, bool>& localMinima, bool debug = false, bool debugRemoveSelfLoops = false, bool debugReassignEdges = false, bool debugRemoveDuplicateEdges = false, bool debugUpdateAllEdgeDistancesFromNode = false, bool debugProcessNegativeEdge = false);

//functions defined in dumb_search.cpp
std::tuple<int, double, std::vector<int>, bool> findMinimumEdgeOrNodeRange(const BGLGraph& bglGraph);

//functions defined in dumb_decimate.cpp
void do_dumb_decimate(BGLGraph& bglGraph, const std::tuple<int, double, std::vector<int>, bool>& localMinima);

// functions defined in graph_debug.cpp
void findAndPrintDuplicateEdges(const BGLGraph& graph);

//functions defined in graph_sample.cpp
std::pair<int, std::unordered_map<int, int>> calculateClusterStatistics(const BGLGraph& bglGraph, int num_threads);
std::unordered_map<std::string, int> calculateEntanglementCounts( const BGLGraph& bglGraph, int num_subsystems, int num_threads);
int calculateVonNeumannEntropy(const std::unordered_map<std::string, int>& entanglementCounts, int subsystems);
std::tuple<int, int> calculateMutualInfo(const std::unordered_map<std::string, int>& entanglementCounts, int subsystems);

//functions defines in graph_persistence
std::set<int> getUniqueClusters(const BGLGraph& bglGraph);
int sampleNodes(const BGLGraph& bglGraph, std::mt19937& gen);
double computeAverageSamples(const BGLGraph& bglGraph, int trials, std::mt19937& gen);

//function defined in graph_persistence -- deprecated
//int chooseRandomNode(const BGLGraph& bglGraph, std::mt19937& gen);
//int getClusterByNode(const BGLGraph& bglGraph, int nodeIndex);
//double averageSpinsForAllClusters(const BGLGraph& bglGraph, std::mt19937& gen);
