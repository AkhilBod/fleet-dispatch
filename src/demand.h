#pragma once

#include <cstdint>
#include <vector>

#include "city.h"

struct Request {
  int id = -1;
  double time = 0;  // seconds since midnight when the rider asks for a car
  int pickup = -1;  // intersection
  int dropoff = -1;
  int zone = -1;    // pickup zone
};

struct DemandConfig {
  double hours = 24;
  // City-wide request rate at the evening peak. The rest of the day is scaled
  // by timeOfDayFactor().
  double peak_requests_per_hour = 2500;
};

// Relative demand at a given hour (0-24). Two rush-hour bumps on a low base,
// plus a small lunch bump. Values are relative; only the shape matters.
double timeOfDayFactor(double hour);

// Relative pickup/dropoff attractiveness of a zone. The middle of the city is
// "downtown" and gets the most trips.
double zoneWeight(const City& city, int zone);

// Generates requests sorted by time with ids 0..n-1. Each zone is an
// independent non-homogeneous Poisson process, sampled by thinning.
std::vector<Request> generateDemand(const City& city, const DemandConfig& config, uint64_t seed);
