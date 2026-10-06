#pragma once

#include <cmath>
#include <cstdint>
#include <random>

// std::mt19937_64 produces the same sequence on every standard library, but
// std::uniform_real_distribution and friends do not. We do the transforms
// ourselves so a seed gives identical results on clang, gcc and MSVC.
class Rng {
 public:
  explicit Rng(uint64_t seed) : engine_(seed) {}

  // Uniform in [0, 1).
  double uniform() { return static_cast<double>(engine_() >> 11) * (1.0 / 9007199254740992.0); }

  // Uniform integer in [0, n).
  int uniformInt(int n) { return static_cast<int>(uniform() * n); }

  // Exponential with the given rate (mean 1 / rate).
  double exponential(double rate) { return -std::log(1.0 - uniform()) / rate; }

 private:
  std::mt19937_64 engine_;
};

// Derive independent-looking seeds from one user seed (splitmix64 step).
inline uint64_t deriveSeed(uint64_t seed, uint64_t stream) {
  uint64_t z = seed + 0x9E3779B97F4A7C15ULL * (stream + 1);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}
