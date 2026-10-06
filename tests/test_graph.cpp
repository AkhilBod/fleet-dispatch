#include <cmath>
#include <limits>

#include "city.h"
#include "graph.h"
#include "harness.h"
#include "rng.h"

// Hand-built graph where the direct edge is slower than a detour:
//   0 --10-- 1
//   |        |
//   1        1
//   |        |
//   2 --1--- 3
TEST(dijkstra_prefers_faster_detour) {
  RoadGraph g(4);
  g.addRoad(0, 1, 10, 100);
  g.addRoad(0, 2, 1, 10);
  g.addRoad(2, 3, 1, 10);
  g.addRoad(3, 1, 1, 10);
  ShortestPathTree t = dijkstra(g, 0);
  CHECK_NEAR(t.seconds[0], 0, 1e-12);
  CHECK_NEAR(t.seconds[1], 3, 1e-12);
  CHECK_NEAR(t.meters[1], 30, 1e-12);
  CHECK_NEAR(t.seconds[3], 2, 1e-12);
  CHECK((extractPath(t, 1) == std::vector<int>{0, 2, 3, 1}));
}

TEST(dijkstra_unreachable_node) {
  RoadGraph g(3);
  g.addRoad(0, 1, 5, 50);
  ShortestPathTree t = dijkstra(g, 0);
  CHECK(std::isinf(t.seconds[2]));
  CHECK(extractPath(t, 2).empty());
  CHECK((extractPath(t, 0) == std::vector<int>{0}));
}

// Compare against Floyd-Warshall on random small graphs.
TEST(dijkstra_matches_floyd_warshall) {
  Rng rng(42);
  const double inf = std::numeric_limits<double>::infinity();
  for (int trial = 0; trial < 50; ++trial) {
    int n = 2 + rng.uniformInt(9);
    RoadGraph g(n);
    std::vector<std::vector<double>> d(n, std::vector<double>(n, inf));
    for (int i = 0; i < n; ++i) d[i][i] = 0;
    int edges = rng.uniformInt(n * 2);
    for (int e = 0; e < edges; ++e) {
      int a = rng.uniformInt(n), b = rng.uniformInt(n);
      double w = 1 + rng.uniformInt(20);
      g.addRoad(a, b, w, w);
      d[a][b] = std::min(d[a][b], w);
      d[b][a] = std::min(d[b][a], w);
    }
    for (int k = 0; k < n; ++k)
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) d[i][j] = std::min(d[i][j], d[i][k] + d[k][j]);

    for (int s = 0; s < n; ++s) {
      ShortestPathTree t = dijkstra(g, s);
      for (int v = 0; v < n; ++v) {
        if (std::isinf(d[s][v])) {
          CHECK(std::isinf(t.seconds[v]));
        } else {
          CHECK_NEAR(t.seconds[v], d[s][v], 1e-9);
          // The reconstructed path must start at s, end at v, and have the claimed length.
          std::vector<int> path = extractPath(t, v);
          CHECK(!path.empty() && path.front() == s && path.back() == v);
          double length = 0;
          for (size_t k = 1; k < path.size(); ++k) {
            double cheapest = inf;
            for (const Edge& e : g.edges(path[k - 1]))
              if (e.to == path[k]) cheapest = std::min(cheapest, e.seconds);
            length += cheapest;
          }
          CHECK_NEAR(length, d[s][v], 1e-9);
        }
      }
    }
  }
}

TEST(grid_without_jitter_is_manhattan) {
  CityConfig cfg;
  cfg.rows = 6;
  cfg.cols = 6;
  cfg.arterial_every = 0;  // no fast roads
  cfg.speed_jitter = 0;
  City city(cfg, 1);
  double edge_seconds = cfg.block_meters / (cfg.street_kmh / 3.6);
  for (int a = 0; a < city.numNodes(); ++a) {
    for (int b = 0; b < city.numNodes(); ++b) {
      int blocks = std::abs(city.rowOf(a) - city.rowOf(b)) + std::abs(city.colOf(a) - city.colOf(b));
      CHECK_NEAR(city.paths().seconds(a, b), blocks * edge_seconds, 1e-6);
    }
  }
}

TEST(arterials_are_faster) {
  CityConfig cfg;
  cfg.speed_jitter = 0;
  City city(cfg, 1);
  // Row 0 is an arterial, row 1 is not. Compare four blocks along each.
  double along_arterial = city.paths().seconds(city.node(0, 0), city.node(0, 4));
  double along_street = city.paths().seconds(city.node(1, 0), city.node(1, 4));
  CHECK(along_arterial < along_street);
  CHECK_NEAR(along_arterial, 4 * cfg.block_meters / (cfg.arterial_kmh / 3.6), 1e-6);
}

TEST(path_cache_reuses_trees) {
  City city(CityConfig{}, 3);
  PathCache& cache = city.paths();
  cache.seconds(5, 100);
  cache.seconds(5, 200);
  cache.path(5, 300);
  CHECK(cache.misses() == 1);
  CHECK(cache.hits() == 2);
  // Symmetric roads mean symmetric travel times.
  CHECK_NEAR(cache.seconds(5, 300), cache.seconds(300, 5), 1e-9);
}
