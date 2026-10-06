#pragma once

#include <cstdint>
#include <vector>

#include "graph.h"

struct CityConfig {
  int rows = 30;               // intersections north-south
  int cols = 30;               // intersections east-west
  double block_meters = 250;   // distance between neighbouring intersections
  double street_kmh = 25;      // typical local street speed
  double arterial_kmh = 50;    // every `arterial_every`-th row and column is faster
  int arterial_every = 5;
  double speed_jitter = 0.15;  // each edge's time is scaled by a random factor in [1-j, 1+j]
  int zone_size = 5;           // zones are zone_size x zone_size blocks of intersections
};

// A grid city: nodes are intersections, edges are road segments. The grid is
// also cut into square zones, which demand generation and rebalancing use.
class City {
 public:
  City(const CityConfig& config, uint64_t seed);
  City(const City&) = delete;
  City& operator=(const City&) = delete;

  const RoadGraph& graph() const { return graph_; }
  PathCache& paths() { return paths_; }
  const CityConfig& config() const { return config_; }

  int numNodes() const { return config_.rows * config_.cols; }
  int node(int row, int col) const { return row * config_.cols + col; }
  int rowOf(int node) const { return node / config_.cols; }
  int colOf(int node) const { return node % config_.cols; }

  int zoneRows() const { return zone_rows_; }
  int zoneCols() const { return zone_cols_; }
  int numZones() const { return zone_rows_ * zone_cols_; }
  int zoneOf(int node) const;
  int zoneCenter(int zone) const;  // an intersection near the middle of the zone
  const std::vector<int>& nodesInZone(int zone) const { return zone_nodes_[zone]; }

 private:
  CityConfig config_;
  RoadGraph graph_;
  PathCache paths_;
  int zone_rows_;
  int zone_cols_;
  std::vector<std::vector<int>> zone_nodes_;
};
