//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"

void BGLGraph::removeOutEdges(int nodeIndex) {
  boost::graph_traits<Graph>::out_edge_iterator ei, eend;
  for (boost::tie(ei, eend) = out_edges(nodeIndex, g); ei != eend;) {
    remove_edge(*ei++, g);
  }
}

void BGLGraph::removeInEdges(int nodeIndex) {
  boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
    remove_edge(*vi, nodeIndex, g);
  }
}

void BGLGraph::addEdge(int from, int to, double distance) {
  EdgeDescriptor e;
  bool inserted;
  boost::tie(e, inserted) = add_edge(from, to, EdgeWeightProperty(distance), g);
}

void BGLGraph::removeEdge(int from, int to) {
  remove_edge(from, to, g);
  remove_edge(to, from, g);
}

void BGLGraph::addNode(int index, double range, NodeStatus status, int clusterIndex, int subsystem) {
  g[index] = Node(range, status, index, clusterIndex, subsystem);
  nodes_by_status[status].push_back(index);
  nodes_by_clusterIndex[clusterIndex].push_back(index);
  nodes_in_subsystem[subsystem].push_back(index);
}

void BGLGraph::updateNodeStatus(int node_index, NodeStatus new_status) {
  NodeStatus old_status = g[node_index].status;
  g[node_index].status = new_status;
  auto &old_list = nodes_by_status[old_status];
  old_list.erase(std::remove(old_list.begin(), old_list.end(),
        node_index), old_list.end());
  nodes_by_status[new_status].push_back(node_index);
  // If the new status is 'Dead', remove all connected edges
  if (new_status == NodeStatus::Dead) {
    removeOutEdges(node_index); // Remove out edges
    removeInEdges(node_index); // Remove in-edges
  }
}

void BGLGraph::updateNodeRange(int node_index, double new_range) {
  g[node_index].range = new_range;
}

void BGLGraph::updateNodeClusterIndex(int node_index, int new_clusterIndex) {
  int old_clusterIndex = g[node_index].clusterIndex;
  g[node_index].clusterIndex = new_clusterIndex;
  auto &old_list = nodes_by_clusterIndex[old_clusterIndex];
  old_list.erase(std::remove(old_list.begin(), old_list.end(),
        node_index), old_list.end());
  nodes_by_clusterIndex[new_clusterIndex].push_back(node_index);
}

void BGLGraph::updateNodeSubsystem(int node_index, int new_subsystem) {
  int old_subsystem = g[node_index].subsystem;
  g[node_index].subsystem = new_subsystem;
  auto &old_list = nodes_in_subsystem[old_subsystem];
  old_list.erase(std::remove(old_list.begin(), old_list.end(),
        node_index), old_list.end());
  nodes_in_subsystem[new_subsystem].push_back(node_index);
}

void BGLGraph::updateEdgeDistance(int from, int to, double new_distance) {
  EdgeDescriptor e;
  bool found;
  boost::tie(e, found) = edge(from, to, g);
  if (found) {
    put(boost::edge_weight_t(), g, e, new_distance);
  } else {
    throw std::runtime_error("No edge found between the specified nodes.");
  }
}
