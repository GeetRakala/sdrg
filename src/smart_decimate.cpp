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
        //bglGraph.updateNodeClusterIndex(nodeInCluster, -1);
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
    if (debug) std::cout << "Updating range of the first node:" << "\n";
    if (debug) std::cout << "firstNodeRange: " << firstNodeRange << "\n";
    if (debug) std::cout << "lastNodeRange: " << lastNodeRange << "\n";
    if (debug) std::cout << "value: " << value << "\n";
    if (debug) std::cout << "newNodeRange: " << newNodeRange << "\n";


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

    //Update distances for intermediate inactive nodes
    //for (size_t index = 1; index < path.size() - 1; ++index) {
     // bglGraph.updateInactiveNeighborEdgeDistances(lastNode,path[index],debugReassignEdges);
     //}

    //update distances for the last node
    if (debug) std::cout << "1. updating path nbr edge distances." << "\n";
    bglGraph.updateNeighborEdgeDistancesLastNode(lastNode, debugReassignEdges);
    if (debug) std::cout << "1. done updating path nbr edge distances." << "\n";

        //update distances for the first node
    if (debug) std::cout << "1b. updating path nbr edge distances." << "\n";
    bglGraph.updateFirstNeighborEdgeDistancesLastNode(lastNode, firstNode, debugReassignEdges);
    if (debug) std::cout << "1b. done updating path nbr edge distances." << "\n";


    //Reassign edges for the last node in the path;
    if (debug) std::cout << "2. reassigning edges for last node." << "\n";
    bglGraph.reassignEdges(firstNode, lastNode, debugReassignEdges);
    if (debug) std::cout << "2. done reassigning edges for last node." << "\n";

    // Reassign edges of all inactive nodes in path
    if (debug) std::cout << "3. reassigning inactive edges for last node." << "\n";
    bglGraph.reassignInactiveEdges(firstNode, path, debugReassignEdges);
    if (debug) std::cout << "3. done reassigning inactive edges for last node." << "\n";

    // Remove self loops from the first node
    if (debug) std::cout << "4. removing self loops from first node." << "\n";
    bglGraph.removeSelfLoops(firstNode, debugRemoveSelfLoops);
    if (debug) std::cout << "4. Self loops removed from node " << firstNode << ".\n";

    // Remove duplicate edges and replace with min distance
    if (debug) std::cout << "5. removing duplicate edges from node." << "\n";
    bglGraph.removeDuplicateEdges(firstNode, debugRemoveDuplicateEdges);
    if (debug) std::cout << "5. Duplicate edges removed from node " << firstNode << ".\n";

    // Set nodes in the path except the first one to dead
    for (size_t index = 1; index < path.size(); ++index) {
      bglGraph.updateNodeStatus(path[index], NodeStatus::Dead);
    }

    if (debug) std::cout << "Edge decimation completed. Nodes set to dead, and cluster index and edges updated.\n";
  }
}
