#pragma once

#include <memory>
#include <string>
#include <vector>

class Simulator;

// A dispatch policy reacts to simulator callbacks and issues assign() /
// reposition() calls on the simulator. All three hooks are optional.
class DispatchPolicy {
 public:
  virtual ~DispatchPolicy() = default;
  virtual std::string name() const = 0;

  // A new ride request just arrived and is in sim.waitingRequests().
  virtual void onRequest(Simulator& sim, int request) { (void)sim, (void)request; }
  // A vehicle just finished a dropoff or a repositioning drive and is idle.
  virtual void onVehicleIdle(Simulator& sim, int vehicle) { (void)sim, (void)vehicle; }
  // Periodic tick, only delivered if tickSeconds() > 0.
  virtual void onTick(Simulator& sim) { (void)sim; }
  virtual double tickSeconds() const { return 0; }
};

// Nearest idle car at request time; a freed car takes the oldest waiting rider it can reach in time.
class GreedyPolicy : public DispatchPolicy {
 public:
  std::string name() const override { return "greedy"; }
  void onRequest(Simulator& sim, int request) override;
  void onVehicleIdle(Simulator& sim, int vehicle) override;
};

// Every batch_seconds, optimally match waiting riders to available cars to
// minimize total pickup ETA (Hungarian algorithm).
class BatchedPolicy : public DispatchPolicy {
 public:
  explicit BatchedPolicy(double batch_seconds = 30) : batch_seconds_(batch_seconds) {}
  std::string name() const override { return "batched"; }
  void onTick(Simulator& sim) override;
  double tickSeconds() const override { return batch_seconds_; }

 private:
  double batch_seconds_;
};

// Batched matching plus moving idle cars toward zones where recent demand
// (a moving average) is higher than the cars already there.
class RebalancingPolicy : public BatchedPolicy {
 public:
  explicit RebalancingPolicy(double batch_seconds = 30) : BatchedPolicy(batch_seconds) {}
  std::string name() const override { return "rebalance"; }
  void onRequest(Simulator& sim, int request) override;
  void onTick(Simulator& sim) override;

  // Expected requests per bin for a zone, averaged over the last few full bins.
  double forecastPerBin(int zone, double now) const;

  static constexpr double kBinSeconds = 300;         // demand counted in 5-minute bins
  static constexpr int kHistoryBins = 6;             // moving average over the last 30 minutes
  static constexpr double kLookaheadBins = 2;        // try to hold 10 minutes of expected demand
  static constexpr double kRebalanceEverySeconds = 120;
  static constexpr double kMaxRepositionSeconds = 900;

 private:
  void rebalance(Simulator& sim);

  std::vector<std::vector<int>> arrivals_;  // arrivals_[bin][zone]
  double next_rebalance_ = 0;
};

// Returns nullptr for an unknown name.
std::unique_ptr<DispatchPolicy> makePolicy(const std::string& name, double batch_seconds = 30);
