#include <algorithm>
#include <map>

#include "demand.h"
#include "harness.h"
#include "metrics.h"
#include "scenario.h"

namespace {

// Midnight through the morning peak with a small fleet. Runs in well under a
// second per policy and is busy enough that riders queue, cars reposition,
// and a good share of riders give up.
ScenarioConfig smallScenario(uint64_t seed) {
  ScenarioConfig cfg;
  cfg.sim.vehicles = 50;
  cfg.sim.hours = 10;
  cfg.sim.seed = seed;
  cfg.demand.peak_requests_per_hour = 900;
  return cfg;
}

const char* kPolicies[] = {"greedy", "batched", "rebalance"};

}  // namespace

TEST(demand_is_sorted_and_valid) {
  City city(CityConfig{}, 1);
  DemandConfig cfg;
  cfg.hours = 24;
  std::vector<Request> requests = generateDemand(city, cfg, 9);
  CHECK(!requests.empty());
  for (size_t i = 0; i < requests.size(); ++i) {
    const Request& r = requests[i];
    CHECK(r.id == static_cast<int>(i));
    CHECK(r.time >= 0 && r.time < 24 * 3600);
    CHECK(r.pickup != r.dropoff);
    CHECK(city.zoneOf(r.pickup) == r.zone);
    if (i > 0) CHECK(requests[i - 1].time <= r.time);
  }
}

TEST(demand_has_rush_hours_and_downtown) {
  City city(CityConfig{}, 1);
  std::vector<Request> requests = generateDemand(city, DemandConfig{}, 11);
  std::vector<int> per_hour(24, 0);
  std::map<double, int> per_weight;
  for (const Request& r : requests) {
    ++per_hour[static_cast<int>(r.time / 3600)];
    ++per_weight[zoneWeight(city, r.zone)];
  }
  CHECK(per_hour[8] > 3 * per_hour[3]);   // morning peak vs. middle of the night
  CHECK(per_hour[18] > 3 * per_hour[3]);  // evening peak
  // 4 downtown zones at weight 4 vs 20 outer zones at weight 1: about 16:20.
  CHECK(per_weight[4.0] > per_weight[1.0] * 0.6);
}

TEST(served_requests_are_picked_up_before_dropoff) {
  for (const char* policy : kPolicies) {
    ScenarioResult result = runScenario(smallScenario(5), policy);
    int completed = 0;
    for (const RequestRecord& rec : result.requests) {
      if (rec.status == RequestStatus::Completed) {
        ++completed;
        CHECK(rec.vehicle >= 0);
        CHECK(rec.assign_time >= rec.request.time);
        CHECK(rec.pickup_time >= rec.assign_time);
        CHECK(rec.dropoff_time > rec.pickup_time);
        CHECK(rec.pickup_time - rec.request.time <= 600 + 1e-6);  // never picked up after patience ran out
      } else {
        // The simulator drains all events, so nothing is left half-done.
        CHECK(rec.status == RequestStatus::Abandoned);
        CHECK(rec.pickup_time < 0 && rec.vehicle == -1);
      }
    }
    CHECK(completed > 0);
    CHECK(completed == result.metrics.served);
    CHECK(result.metrics.abandoned > 0);  // the scenario really is overloaded at the peak
  }
}

TEST(no_vehicle_is_double_booked) {
  for (const char* policy : kPolicies) {
    ScenarioResult result = runScenario(smallScenario(6), policy);
    // Each car's jobs, ordered by assignment time, must not overlap: a car is
    // only given a new rider after dropping off the previous one.
    std::map<int, std::vector<const RequestRecord*>> by_vehicle;
    for (const RequestRecord& rec : result.requests) {
      if (rec.vehicle >= 0) by_vehicle[rec.vehicle].push_back(&rec);
    }
    for (auto& [vehicle, jobs] : by_vehicle) {
      std::sort(jobs.begin(), jobs.end(),
                [](const RequestRecord* a, const RequestRecord* b) { return a->assign_time < b->assign_time; });
      for (size_t k = 1; k < jobs.size(); ++k) CHECK(jobs[k]->assign_time >= jobs[k - 1]->dropoff_time);
    }
  }
}

TEST(event_time_never_goes_backwards) {
  for (const char* policy : kPolicies) {
    CHECK(runScenario(smallScenario(7), policy).time_order_violations == 0);
  }
}

TEST(metrics_are_in_range) {
  for (const char* policy : kPolicies) {
    Metrics m = runScenario(smallScenario(8), policy).metrics;
    CHECK(m.served + m.abandoned == m.requests);
    CHECK(m.utilization >= 0 && m.utilization <= 1);
    CHECK(m.within_cutoff >= 0 && m.within_cutoff <= 1);
    CHECK(m.p95_wait >= 0 && m.p95_wait <= 600 + 1e-6);
    CHECK(m.empty_km > 0);
  }
}

TEST(same_seed_gives_identical_results) {
  for (const char* policy : kPolicies) {
    ScenarioResult a = runScenario(smallScenario(9), policy);
    ScenarioResult b = runScenario(smallScenario(9), policy);
    CHECK(a.requests.size() == b.requests.size());
    for (size_t i = 0; i < a.requests.size() && i < b.requests.size(); ++i) {
      CHECK(a.requests[i].vehicle == b.requests[i].vehicle);
      CHECK(a.requests[i].pickup_time == b.requests[i].pickup_time);
      CHECK(a.requests[i].dropoff_time == b.requests[i].dropoff_time);
    }
    // Solver wall-clock time is the only thing allowed to differ.
    a.metrics.mean_solver_ms = b.metrics.mean_solver_ms = 0;
    a.metrics.max_solver_ms = b.metrics.max_solver_ms = 0;
    CHECK(formatSummary(a.metrics) == formatSummary(b.metrics));
  }
}

TEST(different_seed_gives_different_demand) {
  ScenarioResult a = runScenario(smallScenario(1), "batched");
  ScenarioResult b = runScenario(smallScenario(2), "batched");
  CHECK(a.requests.size() != b.requests.size() || a.requests[0].request.time != b.requests[0].request.time);
}

TEST(percentile_nearest_rank) {
  CHECK_NEAR(percentile({}, 95), 0, 0);
  CHECK_NEAR(percentile({5}, 95), 5, 0);
  std::vector<double> values;
  for (int i = 1; i <= 100; ++i) values.push_back(i);
  CHECK_NEAR(percentile(values, 95), 95, 0);
  CHECK_NEAR(percentile(values, 100), 100, 0);
}
