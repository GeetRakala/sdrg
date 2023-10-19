//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Performs decimation based on the given local minimum.
 *
 * @param bglGraph The BGL graph to modify.
 * @param localMinima The local minima, returned by findLocalMinimum.
 * @param debug Flag to control debug output.
 */
void do_dumb_decimate(BGLGraph& bglGraph, const std::tuple<int, double, std::vector<int>, bool>& localMinima) {

  //debug bool
  bool debug = false;
  int nodeIndex; // node to perform node decimation on
  double value;
  std::vector<int> path;
  bool isNode;
  std::tie(nodeIndex, value, path, isNode) = localMinima;

  if (isNode) {
    //Node decimation

    if (debug) std::cout << "Performing node decimation on node: " << nodeIndex << "\n";
    // Initialize a map to store nodes that belong to the same cluster as the node to be decimated
    std::unordered_map<int, std::vector<int>> nodesToUpdateByCluster;

    // Get the cluster index of the node that needs to be decimated
    int thisNode = nodeIndex;
    int thisNodeClusterIndex = bglGraph.getGraph()[thisNode].clusterIndex;

    if (debug) std::cout << "Cluster index of node " << thisNode << " to be decimated: " << thisNodeClusterIndex << "\n";

    // Find all nodes that belong to the same cluster as the node to be decimated
    auto it = bglGraph.getNodesByClusterIndex().find(thisNodeClusterIndex);
    if (it != bglGraph.getNodesByClusterIndex().end()) {
      nodesToUpdateByCluster[thisNodeClusterIndex] = it->second;
      if (debug) {
        std::cout << "Nodes in the same cluster: ";
        for (const auto& n : it->second) {
          std::cout << n << " ";
        }
        std::cout << "\n";
      }
    } else {
      if (debug) std::cout << "No nodes found in the same cluster.\n";
    }

    // Update cluster index and status for all nodes in the same cluster
    for (const auto& [currentClusterIndex, nodesInCurrentCluster] : nodesToUpdateByCluster) {

      if (debug) std::cout << "Updating cluster index and status for nodes in the cluster: " << currentClusterIndex << "\n";

      for (const auto& nodeInCluster : nodesInCurrentCluster) {
        //bglGraph.updateNodeClusterIndex(nodeInCluster, -1);
        bglGraph.updateNodeStatus(nodeInCluster, NodeStatus::Inactive);
      }
    }

    if (debug) std::cout << "Cluster index and status updated for the node to be decimated: " << thisNode << "\n";

    //reassign edges. Also update edge distances while doing reassignment
    bglGraph.updateEdgesAndRemoveNode(thisNode,debug);
    // remove duplicate edges by setting them to the minimum.
    bglGraph.removeDuplicateEdges(thisNode,debug);

  }

  else {
    // Edge decimation
    if (debug) {
      std::cout << "Performing edge decimation on path: ";
      for (const auto& node : path) std::cout << node << " ";
      std::cout << "\n";
    }

    int lastNode = path.back();
    int firstNode = path[0];
    // Update range of first node
    double firstNodeRange = bglGraph.getGraph()[firstNode].range;
    double lastNodeRange = bglGraph.getGraph()[lastNode].range;
    double newNodeRange = firstNodeRange + lastNodeRange - value;
    bglGraph.updateNodeRange(firstNode,newNodeRange);

    // Get the destination cluster index from the first node in the path
    int firstNodeClusterIndex = bglGraph.getGraph()[firstNode].clusterIndex;
    std::unordered_map<int, std::vector<int>> nodesToUpdateByCluster;

    // Get the cluster index of the last node in the path
    int lastNodeClusterIndex = bglGraph.getGraph()[lastNode].clusterIndex;

    // Find the nodes belonging to this cluster
    auto it = bglGraph.getNodesByClusterIndex().find(lastNodeClusterIndex);
    if (it != bglGraph.getNodesByClusterIndex().end()) {
      nodesToUpdateByCluster[lastNodeClusterIndex] = it->second;
    }

    // Update the cluster index for all the collected nodes
    for (const auto& [currentClusterIndex, nodesInCurrentCluster] : nodesToUpdateByCluster) {
      for (const auto& nodeInCluster : nodesInCurrentCluster) {
        bglGraph.updateNodeClusterIndex(nodeInCluster, firstNodeClusterIndex);
      }
    }

    if (debug) std::cout << "Cluster index updated for last node(and its cluster) in the path.\n";

    // Reassign edges of all nodes in path except first
    for (size_t index = 1; index < path.size(); ++index) {
      bglGraph.reassignEdges(firstNode, path[index],debug);
      if (debug) std::cout << "Edges reassigned from node " << path[index] << " to node " << firstNode << ".\n";
    }

    // Remove self loops from the first node
    bglGraph.removeSelfLoops(firstNode,debug);
    if (debug) std::cout << "Self loops removed from node " << firstNode << ".\n";

    // Remove duplicate edges and replace with min distance
    bglGraph.removeDuplicateEdges(firstNode,debug);
    if (debug) std::cout << "Duplicate edges removed from node " << firstNode << ".\n";
    // Set nodes in the path except the first one to dead
    for (size_t index = 1; index < path.size(); ++index) {
      bglGraph.updateNodeStatus(path[index], NodeStatus::Dead);
    }

    if (debug) std::cout << "Edge decimation completed. Nodes set to dead, and cluster index and edges updated.\n";
  }
}
