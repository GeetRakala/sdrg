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
void do_decimate(BGLGraph& bglGraph, const std::tuple<int, double, std::vector<int>, bool>& localMinima, bool debug, bool debugRemoveSelfLoops, bool debugReassignEdges, bool debugRemoveDuplicateEdges, bool debugUpdateAllEdgeDistancesFromNode, bool debugProcessNegativeEdge) {
  int nodeIndex;
  double value;
  std::vector<int> path;
  bool isNode;
  std::tie(nodeIndex, value, path, isNode) = localMinima;

  if (isNode) {
    // Node decimation
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
        bglGraph.updateNodeClusterIndex(nodeInCluster, -1);
        bglGraph.updateNodeStatus(nodeInCluster, NodeStatus::Inactive);
      }
    }
    // Update the cluster index and status for this node to be decimated
    //bglGraph.updateNodeClusterIndex(thisNode, -1);
    //bglGraph.updateNodeStatus(thisNode, NodeStatus::Inactive);
    if (debug) std::cout << "Cluster index and status updated for the node to be decimated: " << thisNode << "\n";
    // Update distances for all edges originating from this node
    bglGraph.updateAllEdgeDistancesFromNode(thisNode,debugUpdateAllEdgeDistancesFromNode,debugProcessNegativeEdge);
    if (debug) std::cout << "Updated edge distances originating from node: " << thisNode << "\n";
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
    if (debug) std::cout << "Updating range of the first node:" << std::endl;
    if (debug) std::cout << "firstNodeRange: " << firstNodeRange << std::endl;
    if (debug) std::cout << "lastNodeRange: " << lastNodeRange << std::endl;
    if (debug) std::cout << "value: " << value << std::endl;
    if (debug) std::cout << "newNodeRange: " << newNodeRange << std::endl;


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

    // Update the cluster index for the last node in the path
    //bglGraph.updateNodeClusterIndex(lastNode, firstNodeClusterIndex);
    if (debug) std::cout << "Cluster index updated for last node(and its cluster) in the path.\n";

    //update distances for the last node
    bglGraph.updateActiveNeighborEdgeDistances(lastNode, debugReassignEdges);

    //Reassign edges for the last node in the path;
    bglGraph.reassignEdges(firstNode, lastNode, debugReassignEdges);

    // Reassign edges of all inactive nodes in path
    bglGraph.reassignInactiveEdges(firstNode, path, debugReassignEdges);

    // Remove self loops from the first node
    bglGraph.removeSelfLoops(firstNode, debugRemoveSelfLoops);
    if (debug) std::cout << "Self loops removed from node " << firstNode << ".\n";

    // Remove duplicate edges and replace with min distance
    bglGraph.removeDuplicateEdges(firstNode, debugRemoveDuplicateEdges);
    if (debug) std::cout << "Duplicate edges removed from node " << firstNode << ".\n";

    // Set nodes in the path except the first one to dead
    for (size_t index = 1; index < path.size(); ++index) {
      bglGraph.updateNodeStatus(path[index], NodeStatus::Dead);
    }

    if (debug) std::cout << "Edge decimation completed. Nodes set to dead, and cluster index and edges updated.\n";
  }
}
