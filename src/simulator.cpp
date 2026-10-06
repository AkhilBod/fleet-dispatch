#include "simulator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "policy.h"
#include "rng.h"

Simulator::Simulator(City& city, std::vector<Request> requests, const SimConfig& config, DispatchPolicy& policy)
    : city_(city), config_(config), policy_(policy), horizon_(config.hours * 3600.0) {
  // Cars start at random intersections.
  Rng rng(deriveSeed(config.seed, 3));
  vehicles_.resize(config.vehicles);
  for (int i = 0; i < config.vehicles; ++i) {
    vehicles_[i].id = i;
    vehicles_[i].node = rng.uniformInt(city.numNodes());
  }

  records_.reserve(requests.size());
  for (const Request& r : requests) {
    if (r.id != static_cast<int>(records_.size())) throw std::invalid_argument("requests must have ids 0..n-1");
    RequestRecord rec;
    rec.request = r;
    records_.push_back(rec);
    schedule(r.time, EventType::RequestArrival, r.id);
  }
  if (policy_.tickSeconds() > 0) schedule(policy_.tickSeconds(), EventType::DispatchTick, -1);
}

void Simulator::schedule(double time, EventType type, int id, int version) {
  events_.push(Event{time, next_sequence_++, type, id, version});
}

void Simulator::run() {
  while (!events_.empty()) {
    Event e = events_.top();
    events_.pop();
    if (e.time < now_) ++time_order_violations_;
    now_ = e.time;

    switch (e.type) {
      case EventType::RequestArrival:
        onRequestArrival(e.id);
        break;
      case EventType::Abandon:
        onAbandon(e.id);
        break;
      case EventType::DispatchTick:
        onDispatchTick();
        break;
      case EventType::ReachPickup:
      case EventType::ReachDropoff:
      case EventType::ReachRepositionTarget: {
        Vehicle& v = vehicles_[e.id];
        if (e.version != v.version) break;  // the plan this event belonged to was replaced
        if (e.type == EventType::ReachPickup) onReachPickup(v);
        if (e.type == EventType::ReachDropoff) onReachDropoff(v);
        if (e.type == EventType::ReachRepositionTarget) onReachRepositionTarget(v);
        break;
      }
    }
  }
}

// ---------------------------------------------------------------- movement

Leg Simulator::planLeg(int from, int to, double start_time) {
  const ShortestPathTree& tree = city_.paths().from(from);
  Leg leg;
  leg.nodes = extractPath(tree, to);
  if (leg.nodes.empty()) throw std::runtime_error("planLeg: destination unreachable");
  for (int n : leg.nodes) {
    leg.arrive.push_back(start_time + tree.seconds[n]);
    leg.meters.push_back(tree.meters[n]);
  }
  return leg;
}

// The first point where a car could change plans: its current node if idle,
// otherwise the next intersection on its route (we never U-turn mid-block).
Simulator::Stop Simulator::nextStop(const Vehicle& v) const {
  if (v.state == VehicleState::Idle || v.leg.nodes.empty()) return {v.node, now_, 0};
  for (size_t k = 0; k < v.leg.nodes.size(); ++k) {
    if (v.leg.arrive[k] >= now_) return {v.leg.nodes[k], v.leg.arrive[k], static_cast<int>(k)};
  }
  int last = static_cast<int>(v.leg.nodes.size()) - 1;
  return {v.leg.nodes[last], v.leg.arrive[last], last};
}

void Simulator::cutLegAtNextStop(Vehicle& v, Stop stop) {
  if (v.state == VehicleState::Repositioning) v.empty_meters += v.leg.meters[stop.leg_index];
  v.node = stop.node;
  v.leg = Leg{};
  ++v.version;
}

bool Simulator::isAvailable(int vehicle) const {
  VehicleState s = vehicles_[vehicle].state;
  return s == VehicleState::Idle || s == VehicleState::Repositioning;
}

double Simulator::etaSeconds(int vehicle, int node) {
  const Vehicle& v = vehicles_[vehicle];
  if (!isAvailable(vehicle)) throw std::logic_error("etaSeconds: vehicle is busy");
  Stop stop = nextStop(v);
  return (stop.time - now_) + city_.paths().seconds(stop.node, node);
}

bool Simulator::canPickUpInTime(int vehicle, int request) {
  const Request& r = records_[request].request;
  double pickup_at = now_ + etaSeconds(vehicle, r.pickup);
  return pickup_at - r.time <= config_.max_wait + 1e-9;
}

// ---------------------------------------------------------------- actions

void Simulator::assign(int vehicle, int request) {
  Vehicle& v = vehicles_.at(vehicle);
  RequestRecord& rec = records_.at(request);
  if (!isAvailable(vehicle)) throw std::logic_error("assign: vehicle already has a job");
  if (rec.status != RequestStatus::Waiting) throw std::logic_error("assign: request is not waiting");

  Stop stop = nextStop(v);
  cutLegAtNextStop(v, stop);
  v.state = VehicleState::ToPickup;
  v.request = request;
  v.reposition_target = -1;
  v.leg = planLeg(v.node, rec.request.pickup, stop.time);

  rec.status = RequestStatus::Assigned;
  rec.vehicle = vehicle;
  rec.assign_time = now_;
  removeWaiting(request);
  schedule(v.leg.arrive.back(), EventType::ReachPickup, vehicle, v.version);
}

void Simulator::reposition(int vehicle, int target_node) {
  Vehicle& v = vehicles_.at(vehicle);
  if (!isAvailable(vehicle)) throw std::logic_error("reposition: vehicle is busy");
  Stop stop = nextStop(v);
  cutLegAtNextStop(v, stop);
  if (v.node == target_node) {
    v.state = VehicleState::Idle;
    return;
  }
  v.state = VehicleState::Repositioning;
  v.reposition_target = target_node;
  v.leg = planLeg(v.node, target_node, stop.time);
  schedule(v.leg.arrive.back(), EventType::ReachRepositionTarget, vehicle, v.version);
}

void Simulator::recordSolverTime(double milliseconds) { solver_ms_.push_back(milliseconds); }

void Simulator::removeWaiting(int request) {
  auto it = std::find(waiting_.begin(), waiting_.end(), request);
  if (it != waiting_.end()) waiting_.erase(it);
}

double Simulator::clipToHorizon(double t) const { return std::min(t, horizon_); }

// ---------------------------------------------------------------- event handlers

void Simulator::onRequestArrival(int request) {
  waiting_.push_back(request);
  schedule(now_ + config_.max_wait, EventType::Abandon, request);
  policy_.onRequest(*this, request);
}

void Simulator::onAbandon(int request) {
  RequestRecord& rec = records_[request];
  if (rec.status != RequestStatus::Waiting) return;  // already matched
  rec.status = RequestStatus::Abandoned;
  removeWaiting(request);
}

void Simulator::onReachPickup(Vehicle& v) {
  RequestRecord& rec = records_[v.request];
  v.empty_meters += v.leg.meters.back();
  v.node = rec.request.pickup;
  rec.status = RequestStatus::OnBoard;
  rec.pickup_time = now_;

  v.state = VehicleState::WithRider;
  v.leg = planLeg(v.node, rec.request.dropoff, now_);
  schedule(v.leg.arrive.back(), EventType::ReachDropoff, v.id, v.version);
}

void Simulator::onReachDropoff(Vehicle& v) {
  RequestRecord& rec = records_[v.request];
  v.loaded_meters += v.leg.meters.back();
  v.busy_seconds += clipToHorizon(now_) - clipToHorizon(rec.pickup_time);
  v.node = rec.request.dropoff;
  rec.status = RequestStatus::Completed;
  rec.dropoff_time = now_;

  v.state = VehicleState::Idle;
  v.request = -1;
  v.leg = Leg{};
  policy_.onVehicleIdle(*this, v.id);
}

void Simulator::onReachRepositionTarget(Vehicle& v) {
  v.empty_meters += v.leg.meters.back();
  v.node = v.reposition_target;
  v.reposition_target = -1;
  v.state = VehicleState::Idle;
  v.leg = Leg{};
  policy_.onVehicleIdle(*this, v.id);
}

void Simulator::onDispatchTick() {
  policy_.onTick(*this);
  // Keep ticking until the last possible rider has either been matched or given up.
  double next = now_ + policy_.tickSeconds();
  if (next <= horizon_ + config_.max_wait) schedule(next, EventType::DispatchTick, -1);
}

// ---------------------------------------------------------------- results

Metrics Simulator::metrics(const std::string& policy_name) const {
  Metrics m;
  m.policy = policy_name;
  m.vehicles = config_.vehicles;
  m.hours = config_.hours;
  m.requests = static_cast<int>(records_.size());
  m.service_cutoff = config_.service_cutoff;

  std::vector<double> waits;
  int within = 0;
  for (const RequestRecord& rec : records_) {
    if (rec.status == RequestStatus::Abandoned) ++m.abandoned;
    if (rec.pickup_time < 0) continue;
    double wait = rec.pickup_time - rec.request.time;
    waits.push_back(wait);
    if (wait <= config_.service_cutoff) ++within;
  }
  m.served = static_cast<int>(waits.size());
  double sum = 0;
  for (double w : waits) sum += w;
  m.mean_wait = waits.empty() ? 0 : sum / waits.size();
  m.p95_wait = percentile(waits, 95);
  m.within_cutoff = m.requests > 0 ? static_cast<double>(within) / m.requests : 0;

  double busy = 0;
  for (const Vehicle& v : vehicles_) {
    busy += v.busy_seconds;
    m.empty_km += v.empty_meters / 1000.0;
    m.loaded_km += v.loaded_meters / 1000.0;
  }
  if (config_.vehicles > 0 && horizon_ > 0) m.utilization = busy / (config_.vehicles * horizon_);

  m.batches = static_cast<int>(solver_ms_.size());
  for (double ms : solver_ms_) {
    m.mean_solver_ms += ms;
    m.max_solver_ms = std::max(m.max_solver_ms, ms);
  }
  if (m.batches > 0) m.mean_solver_ms /= m.batches;
  return m;
}
