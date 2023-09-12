//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================
#pragma once
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <unordered_map>
#include <vector>
#include <random>
#include <queue>
#include <iostream>
#include <fstream>
#include <algorithm>

struct ConfigParams {
  int seed = 0;
  int latticeSize = 0;
  double theta = 1.0;
  int trials = 0;
  bool history = false;
  bool json = false;
  bool dumb = true;
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

enum class NodeStatus { Dead, Active, Inactive };

struct Node {
  double range;           // Represents 'range' of the node.
  NodeStatus status;      // Status of the node.
  int index;              // Index of the node.
  int clusterIndex;       // Index of the cluster the node belongs to.

  Node() : range(0), status(NodeStatus::Active), index(-1), clusterIndex(-1) {}
  Node(double range, NodeStatus status, int index, int clusterIndex)
    : range(range), status(status), index(index), clusterIndex(clusterIndex) {}
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
  void addNode(int index, double range, NodeStatus status, int clusterIndex);
  void updateNodeStatus(int node_index, NodeStatus new_status);
  void updateNodeRange(int node_index, double new_range);
  void updateNodeClusterIndex(int node_index, int new_clusterIndex);
  void updateEdgeDistance(int from, int to, double new_distance);

  // functions defined in smart_utilities.cpp
  void updateAllEdgeDistancesFromNode(int node_index, bool debug, bool debugProcessNegativeEdge);
  void updateActiveNeighborEdgeDistances(int startNode, bool debug);
  void removeDuplicateEdges(int sourceNode, bool debug);
  void reassignEdges(int newSourceNode, int oldSourceNode, bool debug);
  void reassignInactiveEdges(int newSourceNode, const std::vector<int>& path, bool debug);
  void removeSelfLoops(int sourceNode, bool debug);
  void processNegativeEdge(EdgeDescriptor edgeWithNegativeDistance, double negativeEdgeDistance, bool debug);

  //functions defined in dumb_utilities.cpp
  void updateEdgesAndRemoveNode(int inputNode, bool debug);

};

// functions defined in graph_io.cpp
void generateDataFile(const BGLGraph& bglGraph, const std::string& filename);
void printGraphDetails(const BGLGraph& bglGraph, int L);
bool parseConfig(const std::string& fileName, ConfigParams& params);

// functions defined in graph_init.cpp
void initializeNodeStatus(BGLGraph& graph, std::mt19937& gen);
void initializeRangeAndDistance(BGLGraph& graph, double theta, std::mt19937& gen);
BGLGraph createSquareLatticeGraph(int L, double theta, std::mt19937& gen);

// functions defined in smart_search.cpp
int chooseRandomActiveStartNode(const BGLGraph& graph, std::mt19937& gen);
//std::tuple<int, double, std::vector<int>> ActivatedDijkstra(const BGLGraph& bglGraph, int start, bool debugDijkstra);
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
