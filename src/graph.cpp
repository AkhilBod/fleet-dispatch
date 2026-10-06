#include "graph.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>

void RoadGraph::addRoad(int a, int b, double seconds, double meters) {
  if (a < 0 || b < 0 || a >= numNodes() || b >= numNodes()) throw std::out_of_range("addRoad: bad node");
  if (seconds < 0) throw std::invalid_argument("addRoad: negative travel time");
  adj_[a].push_back({b, seconds, meters});
  adj_[b].push_back({a, seconds, meters});
}

ShortestPathTree dijkstra(const RoadGraph& graph, int source) {
  const int n = graph.numNodes();
  if (source < 0 || source >= n) throw std::out_of_range("dijkstra: bad source");

  const double inf = std::numeric_limits<double>::infinity();
  ShortestPathTree tree;
  tree.source = source;
  tree.seconds.assign(n, inf);
  tree.meters.assign(n, inf);
  tree.parent.assign(n, -1);

  // Min-heap of (distance, node). We allow duplicate entries and skip stale
  // ones when popped, which is simpler than a decrease-key heap.
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> heap;
  tree.seconds[source] = 0.0;
  tree.meters[source] = 0.0;
  heap.push({0.0, source});

  while (!heap.empty()) {
    auto [dist, u] = heap.top();
    heap.pop();
    if (dist > tree.seconds[u]) continue;
    for (const Edge& e : graph.edges(u)) {
      double candidate = dist + e.seconds;
      if (candidate < tree.seconds[e.to]) {
        tree.seconds[e.to] = candidate;
        tree.meters[e.to] = tree.meters[u] + e.meters;
        tree.parent[e.to] = u;
        heap.push({candidate, e.to});
      }
    }
  }
  return tree;
}

std::vector<int> extractPath(const ShortestPathTree& tree, int target) {
  std::vector<int> path;
  if (target < 0 || target >= static_cast<int>(tree.seconds.size())) return path;
  if (tree.seconds[target] == std::numeric_limits<double>::infinity()) return path;
  for (int node = target; node != -1; node = tree.parent[node]) path.push_back(node);
  std::reverse(path.begin(), path.end());
  return path;
}

const ShortestPathTree& PathCache::from(int source) {
  if (source < 0 || source >= static_cast<int>(trees_.size())) throw std::out_of_range("PathCache: bad source");
  if (trees_[source]) {
    ++hits_;
  } else {
    ++misses_;
    trees_[source] = std::make_unique<ShortestPathTree>(dijkstra(graph_, source));
  }
  return *trees_[source];
}
