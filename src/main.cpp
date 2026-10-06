#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "scenario.h"

namespace {

void printUsage() {
  std::puts(
      "usage: dispatch [options]\n"
      "  --policy=greedy|batched|rebalance   dispatch policy (default batched)\n"
      "  --compare                           run all three policies on the same seed, print a markdown table\n"
      "  --vehicles=N                        fleet size (default 300)\n"
      "  --hours=H                           simulated hours starting at midnight (default 24)\n"
      "  --seed=S                            random seed (default 7)\n"
      "  --demand=R                          city-wide requests per hour at the evening peak (default 2500)\n"
      "  --batch=SEC                         batch window for batched/rebalance (default 30)\n"
      "  --max-wait=SEC                      rider patience before giving up (default 600)\n"
      "  --cutoff=SEC                        service-level cutoff for the 'picked up within' metric (default 300)");
}

bool startsWith(const std::string& s, const std::string& prefix) { return s.compare(0, prefix.size(), prefix) == 0; }

}  // namespace

int main(int argc, char** argv) {
  ScenarioConfig config;
  std::string policy = "batched";
  bool compare = false;

  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto value = [&](const std::string& key) { return arg.substr(key.size()); };
      if (arg == "--help" || arg == "-h") {
        printUsage();
        return 0;
      } else if (arg == "--compare") {
        compare = true;
      } else if (startsWith(arg, "--policy=")) {
        policy = value("--policy=");
      } else if (startsWith(arg, "--vehicles=")) {
        config.sim.vehicles = std::stoi(value("--vehicles="));
      } else if (startsWith(arg, "--hours=")) {
        config.sim.hours = std::stod(value("--hours="));
      } else if (startsWith(arg, "--seed=")) {
        config.sim.seed = std::stoull(value("--seed="));
      } else if (startsWith(arg, "--demand=")) {
        config.demand.peak_requests_per_hour = std::stod(value("--demand="));
      } else if (startsWith(arg, "--batch=")) {
        config.batch_seconds = std::stod(value("--batch="));
      } else if (startsWith(arg, "--max-wait=")) {
        config.sim.max_wait = std::stod(value("--max-wait="));
      } else if (startsWith(arg, "--cutoff=")) {
        config.sim.service_cutoff = std::stod(value("--cutoff="));
      } else {
        std::cerr << "unknown argument: " << arg << "\n";
        printUsage();
        return 2;
      }
    }
  } catch (const std::exception&) {
    std::cerr << "could not parse a numeric argument\n";
    return 2;
  }

  if (config.sim.vehicles <= 0 || config.sim.hours <= 0 || config.batch_seconds <= 0) {
    std::cerr << "--vehicles, --hours and --batch must be positive\n";
    return 2;
  }

  try {
    if (compare) {
      std::vector<Metrics> rows;
      for (const char* name : {"greedy", "batched", "rebalance"}) rows.push_back(runScenario(config, name).metrics);
      std::printf("seed %llu, %d vehicles, %.0f h, peak demand %.0f req/h, batch %.0f s, max wait %.0f s\n\n",
                  static_cast<unsigned long long>(config.sim.seed), config.sim.vehicles, config.sim.hours,
                  config.demand.peak_requests_per_hour, config.batch_seconds, config.sim.max_wait);
      std::fputs(formatMarkdownTable(rows).c_str(), stdout);
    } else {
      ScenarioResult result = runScenario(config, policy);
      std::fputs(formatSummary(result.metrics).c_str(), stdout);
    }
  } catch (const std::invalid_argument& e) {
    std::cerr << e.what() << "\n";
    printUsage();
    return 2;
  }
  return 0;
}
