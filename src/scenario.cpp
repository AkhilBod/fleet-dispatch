#include "scenario.h"

#include <stdexcept>

#include "policy.h"
#include "rng.h"

ScenarioResult runScenario(const ScenarioConfig& config, const std::string& policy_name) {
  std::unique_ptr<DispatchPolicy> policy = makePolicy(policy_name, config.batch_seconds);
  if (!policy) throw std::invalid_argument("unknown policy: " + policy_name);

  City city(config.city, deriveSeed(config.sim.seed, 1));
  DemandConfig demand = config.demand;
  demand.hours = config.sim.hours;
  std::vector<Request> requests = generateDemand(city, demand, deriveSeed(config.sim.seed, 2));

  Simulator sim(city, std::move(requests), config.sim, *policy);
  sim.run();

  ScenarioResult result;
  result.metrics = sim.metrics(policy->name());
  result.requests = sim.requests();
  result.time_order_violations = sim.timeOrderViolations();
  return result;
}
