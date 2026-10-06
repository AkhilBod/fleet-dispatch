#include "city.h"

#include <algorithm>
#include <stdexcept>

#include "rng.h"

namespace {

double secondsFor(double meters, double kmh) { return meters / (kmh * 1000.0 / 3600.0); }

}  // namespace

City::City(const CityConfig& config, uint64_t seed)
    : config_(config),
      graph_(config.rows * config.cols),
      paths_(graph_),
      zone_rows_(std::max(1, config.rows / config.zone_size)),
      zone_cols_(std::max(1, config.cols / config.zone_size)) {
  if (config.rows < 2 || config.cols < 2) throw std::invalid_argument("City: grid must be at least 2x2");

  Rng rng(seed);
  auto jitter = [&]() { return 1.0 + config_.speed_jitter * (2.0 * rng.uniform() - 1.0); };
  auto isArterial = [&](int index) { return config_.arterial_every > 0 && index % config_.arterial_every == 0; };

  for (int r = 0; r < config_.rows; ++r) {
    for (int c = 0; c < config_.cols; ++c) {
      // East-west segment along row r: fast if row r is an arterial.
      if (c + 1 < config_.cols) {
        double kmh = isArterial(r) ? config_.arterial_kmh : config_.street_kmh;
        graph_.addRoad(node(r, c), node(r, c + 1), secondsFor(config_.block_meters, kmh) * jitter(),
                       config_.block_meters);
      }
      // North-south segment along column c.
      if (r + 1 < config_.rows) {
        double kmh = isArterial(c) ? config_.arterial_kmh : config_.street_kmh;
        graph_.addRoad(node(r, c), node(r + 1, c), secondsFor(config_.block_meters, kmh) * jitter(),
                       config_.block_meters);
      }
    }
  }

  zone_nodes_.resize(numZones());
  for (int n = 0; n < numNodes(); ++n) zone_nodes_[zoneOf(n)].push_back(n);
}

int City::zoneOf(int node) const {
  int zr = std::min(rowOf(node) / config_.zone_size, zone_rows_ - 1);
  int zc = std::min(colOf(node) / config_.zone_size, zone_cols_ - 1);
  return zr * zone_cols_ + zc;
}

int City::zoneCenter(int zone) const {
  int zr = zone / zone_cols_;
  int zc = zone % zone_cols_;
  int r = std::min(zr * config_.zone_size + config_.zone_size / 2, config_.rows - 1);
  int c = std::min(zc * config_.zone_size + config_.zone_size / 2, config_.cols - 1);
  return node(r, c);
}
