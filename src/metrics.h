#pragma once

#include <string>
#include <vector>

struct Metrics {
  std::string policy;
  int vehicles = 0;
  double hours = 0;
  int requests = 0;
  int served = 0;
  int abandoned = 0;
  double mean_wait = 0;             // seconds, over served riders
  double p95_wait = 0;              // seconds, over served riders
  double service_cutoff = 0;        // seconds
  double within_cutoff = 0;         // fraction of ALL requests picked up within the cutoff
  double utilization = 0;           // fraction of vehicle-time spent carrying a rider
  double empty_km = 0;              // driven without a rider (to pickups and repositioning)
  double loaded_km = 0;
  int batches = 0;                  // solver calls that had work to do
  double mean_solver_ms = 0;
  double max_solver_ms = 0;
};

// Nearest-rank percentile, p in [0, 100]. Returns 0 for an empty list.
double percentile(std::vector<double> values, double p);

std::string formatSummary(const Metrics& m);
std::string formatMarkdownTable(const std::vector<Metrics>& rows);
