#include "metrics.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  size_t rank = static_cast<size_t>(std::ceil(p / 100.0 * values.size()));
  if (rank == 0) rank = 1;
  return values[std::min(rank, values.size()) - 1];
}

namespace {

std::string printf_string(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

std::string printf_string(const char* fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return buffer;
}

double pct(int part, int whole) { return whole > 0 ? 100.0 * part / whole : 0.0; }

}  // namespace

std::string formatSummary(const Metrics& m) {
  std::string s;
  s += printf_string("policy              %s\n", m.policy.c_str());
  s += printf_string("vehicles            %d\n", m.vehicles);
  s += printf_string("simulated hours     %.1f\n", m.hours);
  s += printf_string("requests            %d\n", m.requests);
  s += printf_string("served              %d (%.1f%%)\n", m.served, pct(m.served, m.requests));
  s += printf_string("abandoned           %d (%.1f%%)\n", m.abandoned, pct(m.abandoned, m.requests));
  s += printf_string("mean wait           %.1f s\n", m.mean_wait);
  s += printf_string("p95 wait            %.1f s\n", m.p95_wait);
  s += printf_string("picked up <= %.0f s  %.1f%% of all requests\n", m.service_cutoff, 100 * m.within_cutoff);
  s += printf_string("utilization         %.1f%% of vehicle time carrying riders\n", 100 * m.utilization);
  s += printf_string("empty km            %.0f (%.2f per served trip)\n", m.empty_km,
                     m.served > 0 ? m.empty_km / m.served : 0.0);
  s += printf_string("loaded km           %.0f\n", m.loaded_km);
  s += printf_string("solver batches      %d (mean %.3f ms, max %.3f ms)\n", m.batches, m.mean_solver_ms,
                     m.max_solver_ms);
  return s;
}

std::string formatMarkdownTable(const std::vector<Metrics>& rows) {
  std::string s;
  if (rows.empty()) return s;
  s += printf_string("| Policy | Vehicles | Requests | Served | Mean wait (s) | p95 wait (s) | Picked up <= %.0f s "
                     "| Utilization | Empty km | Solver ms/batch (mean / max) |\n",
                     rows[0].service_cutoff);
  s += "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const Metrics& m : rows) {
    std::string solver = m.batches > 0 ? printf_string("%.3f / %.3f", m.mean_solver_ms, m.max_solver_ms) : "n/a";
    s += printf_string("| %s | %d | %d | %.1f%% | %.1f | %.1f | %.1f%% | %.1f%% | %.0f | %s |\n", m.policy.c_str(),
                       m.vehicles, m.requests, pct(m.served, m.requests), m.mean_wait, m.p95_wait,
                       100 * m.within_cutoff, 100 * m.utilization, m.empty_km, solver.c_str());
  }
  return s;
}
