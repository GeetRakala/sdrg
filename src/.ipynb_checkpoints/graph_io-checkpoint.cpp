//=============================================================
// Copyright 2023 Geet Rakala
// Distributed under the GNU General Public License, Version 3.
// (See accompanying file LICENSE.txt or copy at
// https://www.gnu.org/licenses/gpl-3.0.en.html)
//=============================================================

#include "graph.hpp"
/**
 * @brief Generates a JSON file containing graph data.
 *
 * @param bglGraph Reference to the BGLGraph object containing the graph data.
 * @param filenamePrefix Optional prefix for the file name.
 */
void generateDataFile(const BGLGraph& bglGraph, const std::string& filenamePrefix = "graph_data") {
  static int fileIndex = 1; // Static variable to track the file index
  std::string filename = "json/" + filenamePrefix + "_" + std::to_string(fileIndex++) + ".json";
  const Graph& graph = bglGraph.getGraph(); // Extracting Graph from BGLGraph using getter method

  // Open the file for writing.
  std::ofstream file(filename);

  // Start the JSON structure.
  file << "{\n";
  file << "    \"nodes\": [\n";

  // Total number of nodes in the graph.
  int num_nodes = num_vertices(graph);
  int sqrt_num_nodes = (int)sqrt(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    // Compute x, y positions based on square root of number of nodes.
    file << "        { \"x\": " << i % sqrt_num_nodes << ", \"y\": " << i / sqrt_num_nodes;

    // Output the range of the node.
    file << ", \"range\": " << graph[i].range;

    // Output the cluster index of the node.
    file << ", \"clusterIndex\": " << graph[i].clusterIndex;

    // Output the status of the node using a switch case for clarity.
    file << ", \"status\": ";
    switch (graph[i].status) {
      case NodeStatus::Dead: file << "\"Dead\""; break;
      case NodeStatus::Active: file << "\"Active\""; break;
      case NodeStatus::Inactive: file << "\"Inactive\""; break;
    }

    file << " }";
    // Add a comma if this is not the last node.
    if (i < num_nodes - 1) file << ",";
    file << "\n";
  }

  // End of nodes section, start of edges section.
  file << "    ],\n";
  file << "    \"edges\": [\n";

  boost::property_map<Graph, boost::edge_weight_t>::const_type weightmap = get(boost::edge_weight, graph);


  // Boolean to ensure that the first edge does not have a comma in front of it.
  bool firstEdge = true;

  // Iterate over all the edges in the graph.
  boost::graph_traits<Graph>::edge_iterator ei, ei_end;
  for(boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
    if (!firstEdge) file << ",\n";
    firstEdge = false;

    int from = source(*ei, graph);
    int to = target(*ei, graph);
    double distance = weightmap[*ei]; // Accessing distance using the weightmap


    // Write edge data.
    file << "        { \"from\": " << from << ", \"to\": " << to << ", \"distance\": " << distance << " }";
  }

  // End of the JSON structure.
  file << "\n    ]\n";
  file << "}\n";

  // Close the file.
  file.close();
}

/**
 * @brief Prints the details of the graph, including nodes, edges, and properties.
 *
 * @param bglGraph The BGLGraph object containing the graph details.
 * @param L The lattice size of the graph.
 */
void printGraphDetails(const BGLGraph& bglGraph, int L) {
  const Graph& graph = bglGraph.getGraph();
  for (int i = 0; i < L * L; ++i) {
    std::cout << "Node " << i << " range: " << graph[i].range << " status: ";
    switch (graph[i].status) {
      case NodeStatus::Active: std::cout << "Active"; break;
      case NodeStatus::Inactive: std::cout << "Inactive"; break;
      case NodeStatus::Dead: std::cout << "Dead"; break;
    }
    std::cout << "\nEdges:";
    boost::graph_traits<Graph>::out_edge_iterator ei, eend;
    for (boost::tie(ei, eend) = out_edges(i, graph); ei != eend; ++ei) {
      EdgeDescriptor edge = *ei;
      VertexDescriptor target_vertex = target(edge, graph);
      double distance = get(boost::edge_weight_t(), graph, edge);
      std::cout << " (" << target_vertex << ", " << distance << ")";
    }
    std::cout << "\n";
  }
}

/**
 * @brief Opens and parses the configuration file to retrieve the seed and lattice size.
 *
 * @param fileName The name of the configuration file.
 * @param seed Reference to an integer where the seed value will be stored.
 * @param latticeSize Reference to an integer where the lattice size will be stored.
 * @return True if successful, false otherwise.
 */
bool parseConfig(const std::string& fileName, ConfigParams& params) {
  std::ifstream config(fileName);
  if (!config.is_open()) {
    std::cerr << "Could not open the config file." << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(config, line)) {
    // Ignore comment lines that start with '#'
    if (line[0] != '#') {
      std::istringstream is_line(line);
      std::string key;
      if (std::getline(is_line, key, '=')) {
        std::string value;
        if (std::getline(is_line, value)) {
          if (key == "seed") params.seed = std::stoi(value);
          else if (key == "lattice_size") params.latticeSize = std::stoi(value);
          else if (key == "theta") params.theta = std::stod(value);
          else if (key == "trials") params.trials = std::stoi(value);
          else if (key == "history") params.history = (value == "true");
          else if (key == "json") params.json = (value == "true");
          else if (key == "dumb") params.dumb = (value == "true");
          else if (key == "debug_main") params.debugMain = (value == "true");
          else if (key == "debug_findLocalMinima") params.debugFlm = (value == "true");
          else if (key == "debug_dijkstra") params.debugDijkstra = (value == "true");
          else if (key == "debug_decimate") params.debugDecimate = (value == "true");
          else if (key == "debug_removeSelfLoops") params.debugRemoveSelfLoops = (value == "true");
          else if (key == "debug_reassignEdges") params.debugReassignEdges = (value == "true");
          else if (key == "debug_removeDuplicateEdges") params.debugRemoveDuplicateEdges = (value == "true");
          else if (key == "debug_updateAllEdgeDistancesFromNode") params.debugUpdateAllEdgeDistancesFromNode = (value == "true");
          else if (key == "debug_processNegativeEdge") params.debugProcessNegativeEdge = (value == "true");
        }
      }
    }
  }
  return true;
}
