#pragma once

#include <cstdint>
#include <queue>
#include <string>
#include <vector>

#include "city.h"
#include "demand.h"
#include "metrics.h"

class DispatchPolicy;

enum class VehicleState { Idle, ToPickup, WithRider, Repositioning };
enum class RequestStatus { Waiting, Assigned, OnBoard, Completed, Abandoned };

// A planned drive along a shortest path. arrive[k] is when the car reaches
// nodes[k]; meters[k] is the distance driven from the start to nodes[k].
struct Leg {
  std::vector<int> nodes;
  std::vector<double> arrive;
  std::vector<double> meters;
};

struct Vehicle {
  int id = -1;
  VehicleState state = VehicleState::Idle;
  int node = -1;              // where it is (idle) or where the current leg started
  int request = -1;           // request it is serving, if any
  int reposition_target = -1; // node it is heading to while repositioning
  Leg leg;
  int version = 0;            // bumped when a leg is cut short, so stale events can be ignored
  double empty_meters = 0;
  double loaded_meters = 0;
  double busy_seconds = 0;    // time carrying a rider, clipped to the simulated horizon
};

struct RequestRecord {
  Request request;
  RequestStatus status = RequestStatus::Waiting;
  int vehicle = -1;
  double assign_time = -1;
  double pickup_time = -1;
  double dropoff_time = -1;
};

struct SimConfig {
  int vehicles = 300;
  double hours = 24;
  double max_wait = 600;        // rider patience: unassigned this long means they give up
  double service_cutoff = 300;  // threshold for the "picked up within" metric
  uint64_t seed = 7;
};

enum class EventType { RequestArrival, ReachPickup, ReachDropoff, ReachRepositionTarget, Abandon, DispatchTick };

struct Event {
  double time;
  long sequence;  // tie-breaker so equal-time events run in the order they were scheduled
  EventType type;
  int id;         // request or vehicle id, depending on type
  int version;    // vehicle version when the event was scheduled

  bool operator>(const Event& other) const {
    if (time != other.time) return time > other.time;
    return sequence > other.sequence;
  }
};

// Discrete-event simulator. Holds the fleet and the request book, pops events
// in time order, and calls the dispatch policy at the points where a decision
// can be made. Policies act only through the public methods below.
class Simulator {
 public:
  Simulator(City& city, std::vector<Request> requests, const SimConfig& config, DispatchPolicy& policy);

  void run();

  // ---- Read-only view for policies ----
  double now() const { return now_; }
  City& city() { return city_; }
  const SimConfig& config() const { return config_; }
  const std::vector<Vehicle>& vehicles() const { return vehicles_; }
  const std::vector<RequestRecord>& requests() const { return records_; }
  const std::vector<int>& waitingRequests() const { return waiting_; }  // unassigned, oldest first

  // Idle cars and repositioning cars can take a new job.
  bool isAvailable(int vehicle) const;
  // Seconds from now until the vehicle could be at `node`, using its current plan.
  double etaSeconds(int vehicle, int node);
  // True if the vehicle can reach the pickup before the rider runs out of patience.
  bool canPickUpInTime(int vehicle, int request);

  // ---- Actions for policies ----
  void assign(int vehicle, int request);
  void reposition(int vehicle, int target_node);
  void recordSolverTime(double milliseconds);

  // ---- Results ----
  Metrics metrics(const std::string& policy_name) const;
  int timeOrderViolations() const { return time_order_violations_; }

 private:
  struct Stop {
    int node;
    double time;
    int leg_index;
  };

  void schedule(double time, EventType type, int id, int version = 0);
  Leg planLeg(int from, int to, double start_time);
  Stop nextStop(const Vehicle& v) const;
  void cutLegAtNextStop(Vehicle& v, Stop stop);
  void removeWaiting(int request);
  double clipToHorizon(double t) const;

  void onRequestArrival(int request);
  void onReachPickup(Vehicle& v);
  void onReachDropoff(Vehicle& v);
  void onReachRepositionTarget(Vehicle& v);
  void onAbandon(int request);
  void onDispatchTick();

  City& city_;
  SimConfig config_;
  DispatchPolicy& policy_;
  std::vector<Vehicle> vehicles_;
  std::vector<RequestRecord> records_;
  std::vector<int> waiting_;
  std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events_;
  long next_sequence_ = 0;
  double now_ = 0;
  double horizon_ = 0;
  int time_order_violations_ = 0;
  std::vector<double> solver_ms_;
};
