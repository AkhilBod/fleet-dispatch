#pragma once

#include <string>
#include <vector>

#include "city.h"
#include "demand.h"
#include "metrics.h"
#include "simulator.h"

// Everything needed to reproduce one run.
struct ScenarioConfig {
  CityConfig city;
  DemandConfig demand;
  SimConfig sim;
  double batch_seconds = 30;
};

struct ScenarioResult {
  Metrics metrics;
  std::vector<RequestRecord> requests;
  int time_order_violations = 0;
};

// Builds the city and demand from the seed, runs the named policy, returns results.
// The same config always produces the same city, demand and starting fleet,
// so different policies see identical inputs.
ScenarioResult runScenario(const ScenarioConfig& config, const std::string& policy_name);
