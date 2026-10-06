#include <algorithm>
#include <chrono>
#include <cmath>

#include "assignment.h"
#include "policy.h"
#include "simulator.h"

// ---------------------------------------------------------------- greedy

void GreedyPolicy::onRequest(Simulator& sim, int request) {
  const int pickup = sim.requests()[request].request.pickup;
  int best = -1;
  double best_eta = 0;
  for (const Vehicle& v : sim.vehicles()) {
    if (v.state != VehicleState::Idle) continue;
    double eta = sim.etaSeconds(v.id, pickup);
    if (best == -1 || eta < best_eta) {
      best = v.id;
      best_eta = eta;
    }
  }
  // If even the nearest car is too far, the rider waits for a car to free up nearby.
  if (best != -1 && sim.canPickUpInTime(best, request)) sim.assign(best, request);
}

void GreedyPolicy::onVehicleIdle(Simulator& sim, int vehicle) {
  // Serve the rider who has waited longest among those this car can still reach in time.
  for (int request : sim.waitingRequests()) {
    if (sim.canPickUpInTime(vehicle, request)) {
      sim.assign(vehicle, request);
      return;
    }
  }
}

// ---------------------------------------------------------------- batched

void BatchedPolicy::onTick(Simulator& sim) {
  std::vector<int> riders = sim.waitingRequests();
  std::vector<int> cars;
  for (const Vehicle& v : sim.vehicles()) {
    if (sim.isAvailable(v.id)) cars.push_back(v.id);
  }
  if (riders.empty() || cars.empty()) return;

  // Rows are riders, columns are cars, entries are pickup ETAs in seconds.
  // A pair is forbidden if the car would arrive after the rider gives up, so
  // the solver is free to leave such a rider unmatched for a later batch.
  const double now = sim.now();
  const double max_wait = sim.config().max_wait;
  CostMatrix cost(riders.size(), std::vector<double>(cars.size()));
  for (size_t i = 0; i < riders.size(); ++i) {
    const Request& r = sim.requests()[riders[i]].request;
    for (size_t j = 0; j < cars.size(); ++j) {
      double eta = sim.etaSeconds(cars[j], r.pickup);
      cost[i][j] = (now + eta - r.time <= max_wait + 1e-9) ? eta : kForbidden;
    }
  }

  auto start = std::chrono::steady_clock::now();
  std::vector<int> match = solveAssignment(cost);
  auto stop = std::chrono::steady_clock::now();
  sim.recordSolverTime(std::chrono::duration<double, std::milli>(stop - start).count());

  for (size_t i = 0; i < riders.size(); ++i) {
    int j = match[i];
    if (j >= 0 && cost[i][j] < kForbidden) sim.assign(cars[j], riders[i]);
  }
}

// ---------------------------------------------------------------- rebalancing

void RebalancingPolicy::onRequest(Simulator& sim, int request) {
  const Request& r = sim.requests()[request].request;
  size_t bin = static_cast<size_t>(r.time / kBinSeconds);
  if (arrivals_.size() <= bin) arrivals_.resize(bin + 1, std::vector<int>(sim.city().numZones(), 0));
  ++arrivals_[bin][r.zone];
}

double RebalancingPolicy::forecastPerBin(int zone, double now) const {
  int current = static_cast<int>(now / kBinSeconds);
  int total = 0;
  int bins = 0;
  for (int b = current - kHistoryBins; b < current; ++b) {
    if (b < 0) continue;
    ++bins;
    if (b < static_cast<int>(arrivals_.size())) total += arrivals_[b][zone];
  }
  return bins > 0 ? static_cast<double>(total) / bins : 0.0;
}

void RebalancingPolicy::onTick(Simulator& sim) {
  BatchedPolicy::onTick(sim);  // match riders first; only leftover idle cars get moved
  if (sim.now() >= next_rebalance_) {
    rebalance(sim);
    next_rebalance_ = sim.now() + kRebalanceEverySeconds;
  }
}

void RebalancingPolicy::rebalance(Simulator& sim) {
  City& city = sim.city();
  const int zones = city.numZones();

  // Target: enough cars to cover the next few minutes of forecast demand.
  std::vector<int> target(zones), supply(zones, 0);
  for (int z = 0; z < zones; ++z) target[z] = static_cast<int>(std::lround(forecastPerBin(z, sim.now()) * kLookaheadBins));

  // Supply: idle cars in the zone plus cars already driving there.
  std::vector<std::vector<int>> idle_in_zone(zones);
  for (const Vehicle& v : sim.vehicles()) {
    if (v.state == VehicleState::Idle) {
      int z = city.zoneOf(v.node);
      ++supply[z];
      idle_in_zone[z].push_back(v.id);
    } else if (v.state == VehicleState::Repositioning) {
      ++supply[city.zoneOf(v.reposition_target)];
    }
  }

  // Donors: idle cars in zones that have more than they need.
  std::vector<int> donors;
  for (int z = 0; z < zones; ++z) {
    int spare = std::min(static_cast<int>(idle_in_zone[z].size()), supply[z] - target[z]);
    for (int k = 0; k < spare; ++k) donors.push_back(idle_in_zone[z][k]);
  }

  // Fill the biggest shortfalls first, each with the closest donor.
  std::vector<int> short_zones;
  for (int z = 0; z < zones; ++z) {
    if (target[z] > supply[z]) short_zones.push_back(z);
  }
  std::sort(short_zones.begin(), short_zones.end(), [&](int a, int b) {
    int da = target[a] - supply[a], db = target[b] - supply[b];
    return da != db ? da > db : a < b;
  });

  int budget = std::max(1, static_cast<int>(sim.vehicles().size()) / 10);  // cap moves per round
  for (int z : short_zones) {
    int center = city.zoneCenter(z);
    for (int need = target[z] - supply[z]; need > 0 && budget > 0 && !donors.empty(); --need) {
      size_t best = 0;
      double best_eta = sim.etaSeconds(donors[0], center);
      for (size_t k = 1; k < donors.size(); ++k) {
        double eta = sim.etaSeconds(donors[k], center);
        if (eta < best_eta) {
          best = k;
          best_eta = eta;
        }
      }
      if (best_eta > kMaxRepositionSeconds) break;
      sim.reposition(donors[best], center);
      donors.erase(donors.begin() + best);
      --budget;
    }
  }
}

// ---------------------------------------------------------------- factory

std::unique_ptr<DispatchPolicy> makePolicy(const std::string& name, double batch_seconds) {
  if (name == "greedy") return std::make_unique<GreedyPolicy>();
  if (name == "batched") return std::make_unique<BatchedPolicy>(batch_seconds);
  if (name == "rebalance") return std::make_unique<RebalancingPolicy>(batch_seconds);
  return nullptr;
}
