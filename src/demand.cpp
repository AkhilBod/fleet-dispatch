#include "demand.h"

#include <algorithm>
#include <cmath>

#include "rng.h"

namespace {

double bump(double hour, double center, double width) {
  double z = (hour - center) / width;
  return std::exp(-0.5 * z * z);
}

int randomNodeIn(const City& city, int zone, Rng& rng) {
  const std::vector<int>& nodes = city.nodesInZone(zone);
  return nodes[rng.uniformInt(static_cast<int>(nodes.size()))];
}

}  // namespace

double timeOfDayFactor(double hour) {
  hour = std::fmod(hour, 24.0);
  return 0.2 + 1.0 * bump(hour, 8.5, 1.2) + 1.2 * bump(hour, 18.0, 1.6) + 0.35 * bump(hour, 13.0, 2.0);
}

double zoneWeight(const City& city, int zone) {
  double center_r = (city.zoneRows() - 1) / 2.0;
  double center_c = (city.zoneCols() - 1) / 2.0;
  double dr = std::abs(zone / city.zoneCols() - center_r);
  double dc = std::abs(zone % city.zoneCols() - center_c);
  double ring = std::max(dr, dc);  // 0.5 for the central 2x2 zones of a 6x6 layout
  if (ring < 1.0) return 4.0;
  if (ring < 2.0) return 2.0;
  return 1.0;
}

std::vector<Request> generateDemand(const City& city, const DemandConfig& config, uint64_t seed) {
  Rng rng(seed);
  const int zones = city.numZones();

  // Find the peak of the daily profile so we can scale it to the requested rate.
  double peak_factor = 0;
  for (double h = 0; h < 24.0; h += 0.01) peak_factor = std::max(peak_factor, timeOfDayFactor(h));

  std::vector<double> weights(zones);
  double total_weight = 0;
  for (int z = 0; z < zones; ++z) total_weight += weights[z] = zoneWeight(city, z);

  // Cumulative weights for choosing a dropoff zone.
  std::vector<double> cumulative(zones);
  double running = 0;
  for (int z = 0; z < zones; ++z) cumulative[z] = (running += weights[z] / total_weight);

  const double horizon = config.hours * 3600.0;
  std::vector<Request> requests;

  for (int z = 0; z < zones; ++z) {
    // Rate in requests per second for this zone at the busiest moment of the day.
    double max_rate = config.peak_requests_per_hour / 3600.0 * weights[z] / total_weight;
    if (max_rate <= 0) continue;
    // Thinning: draw candidates at the max rate, keep each with probability
    // rate(t) / max_rate. The kept points form a Poisson process with rate(t).
    double t = 0;
    while (true) {
      t += rng.exponential(max_rate);
      if (t >= horizon) break;
      double keep_probability = timeOfDayFactor(t / 3600.0) / peak_factor;
      if (rng.uniform() >= keep_probability) continue;

      Request r;
      r.time = t;
      r.zone = z;
      r.pickup = randomNodeIn(city, z, rng);
      do {
        double u = rng.uniform();
        int dz = static_cast<int>(std::lower_bound(cumulative.begin(), cumulative.end(), u) - cumulative.begin());
        r.dropoff = randomNodeIn(city, std::min(dz, zones - 1), rng);
      } while (r.dropoff == r.pickup);
      requests.push_back(r);
    }
  }

  std::stable_sort(requests.begin(), requests.end(),
                   [](const Request& a, const Request& b) { return a.time < b.time; });
  for (size_t i = 0; i < requests.size(); ++i) requests[i].id = static_cast<int>(i);
  return requests;
}
