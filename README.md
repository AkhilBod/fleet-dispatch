# fleet-dispatch

A small discrete-event simulator for a robotaxi fleet in a grid city. It
generates a day of ride requests, runs a fleet of cars against them, and
compares three ways of deciding which car picks up which rider:

1. **greedy**: when a request arrives, send the nearest idle car.
2. **batched**: every 30 seconds, match all waiting riders to all available
   cars at once so the total pickup ETA is as small as possible (Hungarian
   algorithm).
3. **rebalance**: batched matching, plus moving idle cars toward zones where
   recent demand is higher than the number of cars already there.

It is plain C++17 with no dependencies. The whole thing is about 1,900 lines
including tests and is meant to be read in one sitting.

## Build and run

```
cmake -B build && cmake --build build
ctest --test-dir build            # or ./build/fleet_tests for per-test output
```

```
./build/dispatch --policy=batched --vehicles=300 --hours=24 --seed=7
./build/dispatch --compare --vehicles=200
./build/dispatch --help
```

Other flags: `--demand=R` (city-wide requests per hour at the evening peak,
default 2500), `--batch=SEC` (batch window, default 30), `--max-wait=SEC`
(rider patience, default 600), `--cutoff=SEC` (threshold for the service-level
metric, default 300). A 24-hour run takes well under a second.

## Design

### City (`src/city.*`, `src/graph.*`)

The city is a 30 x 30 grid of intersections, 250 m apart. Every road is two-way.
Local streets run at 25 km/h and every fifth row and column is an arterial at
50 km/h. Each segment's travel time is scaled by a random factor in [0.85, 1.15]
so that routes are not all ties. The grid is cut into 36 zones of 5 x 5
intersections, which the demand model and the rebalancer use.

Shortest paths use textbook Dijkstra with a binary heap and lazy deletion
(stale heap entries are skipped when popped). `PathCache` runs Dijkstra at most
once per source node and keeps the whole tree (travel time, distance, and parent
pointers). With 900 nodes, caching every source is a few MB, so after warm-up
every ETA lookup is an array read and a path is a walk up the parent pointers.

### Demand (`src/demand.*`)

Each zone is an independent Poisson process whose rate changes over the day.
The daily shape (`timeOfDayFactor`) is a low base plus Gaussian bumps at 8:30
(morning peak), 13:00 (small lunch bump) and 18:00 (evening peak, the largest).
Zones near the middle of the map are weighted 4x (downtown), the ring around
them 2x, and the outer ring 1x.

A Poisson process with a time-varying rate is sampled by *thinning*: draw
candidate arrivals at the zone's maximum rate, then keep each one with
probability `rate(t) / max_rate`. Pickup is a random intersection in the zone;
the dropoff zone is drawn with the same downtown weighting.

Random numbers come from `std::mt19937_64` with hand-written uniform and
exponential transforms (`src/rng.h`). The standard library's distributions are
allowed to differ between compilers; the engine is not, so a seed gives the
same day everywhere.

### Simulator (`src/simulator.*`)

The simulator is a priority queue of events ordered by time, with a sequence
number to break ties so equal-time events run in the order they were
scheduled. Event types:

- `RequestArrival`: the rider joins the waiting list and an `Abandon` event is
  scheduled `max_wait` seconds later.
- `ReachPickup`, `ReachDropoff`, `ReachRepositionTarget`: a car finished a leg.
- `Abandon`: if the rider is still unmatched, they give up.
- `DispatchTick`: periodic call into the policy (batched and rebalance only).

Cars are `Idle`, `ToPickup`, `WithRider`, or `Repositioning`. A car's plan is a
`Leg`: the node list of a shortest path plus the time it reaches each node.
Positions are never stepped forward tick by tick; the simulator only needs to
know where a car will be when something happens.

Repositioning cars can be given a rider mid-drive. In that case the car first
finishes the block it is on (it reaches the next intersection on its route),
then drives to the pickup. Interrupting a leg bumps the car's `version`
counter. Every car event carries the version it was scheduled with, so the
old `ReachRepositionTarget` event is ignored when it pops. This is a simple
way to "cancel" events in a priority queue that does not support removal.

The simulator stops generating requests at the end of the horizon but keeps
running until every trip finishes and every rider has been matched or given
up. Utilization only counts time inside the horizon.

### Policies (`src/policy.h`, `src/policies.cpp`)

A policy implements up to three hooks: `onRequest`, `onVehicleIdle`, and
`onTick`. It acts only through `Simulator::assign` and `Simulator::reposition`,
which check that the car is free and the rider is still waiting (and throw
otherwise). All policies share one rule: a car is only sent if it can arrive
within the rider's `max_wait`, so no policy ever drives to a rider who will
have left.

- **Greedy.** On arrival, the nearest idle car takes the request if it can make
  it in time; otherwise the rider waits. When a car frees up, it takes the
  oldest waiting rider it can still reach in time.
- **Batched.** Every 30 s it builds a cost matrix with one row per waiting
  rider and one column per available car (idle or repositioning). Entries are
  pickup ETAs. Pairs where the car would arrive after the rider gives up get a
  large `kForbidden` cost. The matrix is solved with the Hungarian algorithm and
  forbidden pairs in the result are dropped, so those riders stay unmatched and
  are tried again next batch.
- **Rebalance.** Batched matching, then every 2 minutes: count arrivals per
  zone in 5-minute bins, forecast each zone's next bin as the average of the
  last six bins (a 30-minute moving average), and set a target of two bins'
  worth of cars. Supply is idle cars in the zone plus cars already
  repositioning there. Idle cars in zones above target are donors; the zones
  furthest below target are filled first, each with the closest donor, capped
  at 10% of the fleet per round and 15 minutes of driving per move.

### Assignment solver (`src/assignment.*`)

The Hungarian algorithm in its O(n^2 m) form with row and column potentials:
add rows one at a time, and for each new row run a Dijkstra-like search over
reduced costs `cost[i][j] - u[i] - v[j]` to find the cheapest augmenting path,
updating potentials so that matched edges stay at zero reduced cost. If there
are more rows than columns, the solver works on the transpose.

`kForbidden` is 1e7 seconds, larger than any possible sum of real ETAs in a
batch, so the solver first maximizes the number of allowed pairs and then
minimizes their total ETA. That is the objective the brute-force test checks.

### Metrics (`src/metrics.*`)

- Mean and p95 (nearest rank) wait, over riders who were picked up.
- Share of *all* requests picked up within the cutoff (default 5 minutes), so
  abandoned riders count against it.
- Utilization: share of vehicle-time inside the horizon spent carrying a rider.
- Empty km: distance driven without a rider, to pickups or repositioning.
- Solver time: wall-clock time of each Hungarian call that had at least one
  rider and one car. This is the only number that changes between runs.

## Results

Produced by `./build/dispatch --compare --vehicles=200` and
`--vehicles=400` on an Apple Silicon Mac, default settings, seed 7. Solver
times are wall clock and will differ on your machine and between runs; every
other column is deterministic.

```
seed 7, 200 vehicles, 24 h, peak demand 2500 req/h, batch 30 s, max wait 600 s

| Policy | Vehicles | Requests | Served | Mean wait (s) | p95 wait (s) | Picked up <= 300 s | Utilization | Empty km | Solver ms/batch (mean / max) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| greedy | 200 | 25843 | 89.3% | 326.7 | 593.6 | 40.9% | 45.8% | 26529 | n/a |
| batched | 200 | 25843 | 92.2% | 149.5 | 438.0 | 81.1% | 47.6% | 15874 | 0.003 / 0.025 |
| rebalance | 200 | 25843 | 92.1% | 147.8 | 444.4 | 80.5% | 47.6% | 18561 | 0.003 / 0.022 |
```

```
seed 7, 400 vehicles, 24 h, peak demand 2500 req/h, batch 30 s, max wait 600 s

| Policy | Vehicles | Requests | Served | Mean wait (s) | p95 wait (s) | Picked up <= 300 s | Utilization | Empty km | Solver ms/batch (mean / max) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| greedy | 400 | 25843 | 100.0% | 52.5 | 134.6 | 99.9% | 25.9% | 12876 | n/a |
| batched | 400 | 25843 | 100.0% | 65.2 | 144.8 | 99.9% | 25.9% | 12224 | 0.005 / 0.075 |
| rebalance | 400 | 25843 | 100.0% | 56.8 | 120.4 | 100.0% | 25.9% | 16838 | 0.004 / 0.034 |
```

What I take from these:

- **Batching matters most when the fleet is short.** With 200 cars the fleet is
  saturated at the evening peak. Greedy sends whichever car frees up to the
  rider who has waited longest, often across town, so cars spend their time
  driving empty to riders who are already late. Batching looks at everyone
  together and pairs riders with nearby cars, which halves the mean wait,
  doubles the share picked up within 5 minutes, and cuts empty km by about 40%.
- **With plenty of cars, greedy wins on mean wait.** With 400 cars there is
  almost always an idle car nearby, and the batched policy's main cost is that
  a rider waits up to 30 s for the next batch before anything happens. Batching
  still ends up with slightly fewer empty km.
- **Rebalancing helps the tail when there are spare cars, and does nothing
  when there are none.** At 400 cars it brings p95 wait from 145 s to 120 s at
  the cost of almost 40% more empty km. At 200 cars there are almost no idle
  cars to move during the peaks, so it only adds empty driving.
- **The matching problem is small.** Solver calls take microseconds because each
  batch only sees riders who arrived in the last 30 s (plus a backlog) and cars
  free at that moment. Even saturated, the matrices are tens by tens, not
  hundreds by hundreds.
- Utilization only counts time with a rider on board, so it follows the
  number of trips served. It is lower for greedy at 200 cars because greedy
  serves fewer riders, not because its cars are less busy; they are busy
  driving empty.
- **Greedy is fragile near capacity.** Running seeds 1 to 3 gives the same
  ordering at 200 cars. At 400 cars, seeds 1 and 2 also match the table above,
  but on seed 3 greedy tips over during the evening peak: mean wait 98.9 s,
  p95 456.8 s, 89.3% within 5 minutes and 23,203 empty km, against 67.1 s,
  150.6 s, 99.8% and 12,487 km for batched. Once a few riders are left waiting,
  freed cars are sent far away to serve them, which keeps more riders waiting.
  The batched policies did not show this on any seed tried.

## Tests

`tests/` has a tiny harness (`harness.h`, about 50 lines) and 20 tests:

- Dijkstra on hand-built graphs (detour beats direct edge, unreachable nodes),
  against Floyd-Warshall on 50 random graphs including path length, Manhattan
  distances on a uniform grid, arterials faster than streets, cache hits.
- Hungarian against brute-force enumeration on 900 random matrices: square,
  rectangular both ways, and with 40% forbidden pairs.
- Demand: sorted, valid, peaks larger than the night, downtown busier.
- Simulator, for all three policies on a small overloaded scenario: every
  completed rider is assigned, picked up, then dropped off in that order and
  within patience; everyone else is marked abandoned; no car gets a new rider
  before dropping off the previous one; event time never goes backwards; the
  same seed produces identical per-request results.

## Limitations

These are deliberate simplifications, and some of them change the conclusions.

- Travel times are fixed. There is no congestion, no traffic signals, and cars
  do not slow each other down.
- Riders are never pooled; a car carries one request at a time.
- A car that is already driving to a pickup is never reassigned, even if a
  better match appears.
- Riders never cancel after being matched, and patience is the same for
  everyone.
- The demand model is synthetic. The shape (two rush hours, a downtown) is
  plausible, but the numbers are not fitted to any real city. Destinations do
  not depend on time of day, so there is no "everyone goes downtown in the
  morning" flow, which is where rebalancing would usually matter most.
- The forecast is a moving average and lags behind the ramp-up of each peak.
- The rebalancer is a greedy heuristic, not an optimization, and its parameters
  (bin size, lookahead, move cap) were picked by hand, not tuned.
- Results are for one seed. Differences of a few percent between policies
  should be checked across several seeds before drawing conclusions.
- No charging, shift changes, or vehicle downtime.
