#include <algorithm>
#include <limits>

#include "assignment.h"
#include "harness.h"
#include "rng.h"

namespace {

// Try every way of matching min(rows, cols) pairs and return the cheapest total.
void bruteForce(const CostMatrix& c, size_t row, std::vector<bool>& used_cols, int skips_left, double so_far,
                double& best) {
  if (row == c.size()) {
    best = std::min(best, so_far);
    return;
  }
  for (size_t col = 0; col < c[0].size(); ++col) {
    if (used_cols[col]) continue;
    used_cols[col] = true;
    bruteForce(c, row + 1, used_cols, skips_left, so_far + c[row][col], best);
    used_cols[col] = false;
  }
  if (skips_left > 0) bruteForce(c, row + 1, used_cols, skips_left - 1, so_far, best);
}

double bruteForceOptimum(const CostMatrix& c) {
  std::vector<bool> used(c[0].size(), false);
  int skips = std::max(0, static_cast<int>(c.size()) - static_cast<int>(c[0].size()));
  double best = std::numeric_limits<double>::infinity();
  bruteForce(c, 0, used, skips, 0, best);
  return best;
}

CostMatrix randomMatrix(Rng& rng, int rows, int cols, double forbidden_share) {
  CostMatrix c(rows, std::vector<double>(cols));
  for (auto& row : c)
    for (double& x : row) x = rng.uniform() < forbidden_share ? kForbidden : 1 + rng.uniformInt(600);
  return c;
}

// The answer must be a valid matching of exactly min(rows, cols) pairs.
bool isValidMatching(const CostMatrix& c, const std::vector<int>& m) {
  if (m.size() != c.size()) return false;
  std::vector<bool> used(c[0].size(), false);
  size_t matched = 0;
  for (int col : m) {
    if (col < 0) continue;
    if (col >= static_cast<int>(c[0].size()) || used[col]) return false;
    used[col] = true;
    ++matched;
  }
  return matched == std::min(c.size(), c[0].size());
}

}  // namespace

TEST(hungarian_known_example) {
  CostMatrix c = {{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
  std::vector<int> m = solveAssignment(c);
  CHECK_NEAR(assignmentCost(c, m), 5, 1e-9);  // 1 + 2 + 2
  CHECK((m == std::vector<int>{1, 0, 2}));
}

TEST(hungarian_matches_brute_force_square) {
  Rng rng(1);
  for (int trial = 0; trial < 300; ++trial) {
    int n = 1 + rng.uniformInt(7);
    CostMatrix c = randomMatrix(rng, n, n, 0.0);
    std::vector<int> m = solveAssignment(c);
    CHECK(isValidMatching(c, m));
    CHECK_NEAR(assignmentCost(c, m), bruteForceOptimum(c), 1e-6);
  }
}

TEST(hungarian_matches_brute_force_rectangular) {
  Rng rng(2);
  for (int trial = 0; trial < 300; ++trial) {
    int rows = 1 + rng.uniformInt(7);
    int cols = 1 + rng.uniformInt(7);
    CostMatrix c = randomMatrix(rng, rows, cols, 0.0);
    std::vector<int> m = solveAssignment(c);
    CHECK(isValidMatching(c, m));
    CHECK_NEAR(assignmentCost(c, m), bruteForceOptimum(c), 1e-6);
  }
}

// With forbidden pairs the solver should first maximize the number of allowed
// pairs and then minimize their cost. Brute force over the same matrix has the
// same objective because kForbidden dwarfs any sum of real costs.
TEST(hungarian_with_forbidden_pairs) {
  Rng rng(3);
  for (int trial = 0; trial < 300; ++trial) {
    int rows = 1 + rng.uniformInt(6);
    int cols = 1 + rng.uniformInt(6);
    CostMatrix c = randomMatrix(rng, rows, cols, 0.4);
    std::vector<int> m = solveAssignment(c);
    CHECK(isValidMatching(c, m));
    CHECK_NEAR(assignmentCost(c, m), bruteForceOptimum(c), 1e-3);
  }
}

TEST(hungarian_empty_inputs) {
  CHECK(solveAssignment(CostMatrix{}).empty());
  CostMatrix no_cols(3);
  CHECK((solveAssignment(no_cols) == std::vector<int>{-1, -1, -1}));
}
