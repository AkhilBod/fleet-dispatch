#pragma once

#include <memory>
#include <vector>

struct Edge {
  int to;
  double seconds;  // travel time
  double meters;   // physical length, used for empty-mile accounting
};

// Undirected road network stored as an adjacency list.
class RoadGraph {
 public:
  explicit RoadGraph(int num_nodes) : adj_(num_nodes) {}

  // Adds a two-way road between a and b.
  void addRoad(int a, int b, double seconds, double meters);

  int numNodes() const { return static_cast<int>(adj_.size()); }
  const std::vector<Edge>& edges(int node) const { return adj_[node]; }

 private:
  std::vector<std::vector<Edge>> adj_;
};

// Result of one Dijkstra run: fastest route from `source` to every node.
struct ShortestPathTree {
  int source = -1;
  std::vector<double> seconds;  // infinity if unreachable
  std::vector<double> meters;   // length of the fastest route
  std::vector<int> parent;      // previous node on the route, -1 at source
};

ShortestPathTree dijkstra(const RoadGraph& graph, int source);

// Node list from tree.source to target, both ends included. Empty if unreachable.
std::vector<int> extractPath(const ShortestPathTree& tree, int target);

// Lazily runs Dijkstra once per source node and keeps the tree. The city has
// 900 nodes, so caching every source costs a few MB and makes later lookups O(1).
class PathCache {
 public:
  explicit PathCache(const RoadGraph& graph) : graph_(graph), trees_(graph.numNodes()) {}

  const ShortestPathTree& from(int source);
  double seconds(int a, int b) { return from(a).seconds[b]; }
  double meters(int a, int b) { return from(a).meters[b]; }
  std::vector<int> path(int a, int b) { return extractPath(from(a), b); }

  long hits() const { return hits_; }
  long misses() const { return misses_; }

 private:
  const RoadGraph& graph_;
  std::vector<std::unique_ptr<ShortestPathTree>> trees_;
  long hits_ = 0;
  long misses_ = 0;
};
