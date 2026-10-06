#include "assignment.h"

#include <limits>
#include <stdexcept>

namespace {

// Core solver for rows <= cols. Uses 1-based indexing internally, following the
// well-known formulation with row potentials u, column potentials v, and a
// Dijkstra-like search for an augmenting path for each new row.
std::vector<int> hungarianRowsLeqCols(const CostMatrix& a) {
  const int n = static_cast<int>(a.size());
  const int m = static_cast<int>(a[0].size());
  const double inf = std::numeric_limits<double>::infinity();

  std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
  std::vector<int> match_of_col(m + 1, 0);  // row matched to column j (0 = none)
  std::vector<int> way(m + 1, 0);           // previous column on the augmenting path

  for (int i = 1; i <= n; ++i) {
    // Add row i. Column 0 is a virtual column holding the new row.
    match_of_col[0] = i;
    int j0 = 0;
    std::vector<double> min_slack(m + 1, inf);
    std::vector<bool> used(m + 1, false);
    do {
      used[j0] = true;
      int i0 = match_of_col[j0];
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) continue;
        double reduced = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (reduced < min_slack[j]) {
          min_slack[j] = reduced;
          way[j] = j0;
        }
        if (min_slack[j] < delta) {
          delta = min_slack[j];
          j1 = j;
        }
      }
      // Shift potentials so the cheapest unused column becomes tight.
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[match_of_col[j]] += delta;
          v[j] -= delta;
        } else {
          min_slack[j] -= delta;
        }
      }
      j0 = j1;
    } while (match_of_col[j0] != 0);  // stop when we reach a free column

    // Flip the augmenting path.
    do {
      int j1 = way[j0];
      match_of_col[j0] = match_of_col[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> row_to_col(n, -1);
  for (int j = 1; j <= m; ++j) {
    if (match_of_col[j] != 0) row_to_col[match_of_col[j] - 1] = j - 1;
  }
  return row_to_col;
}

}  // namespace

std::vector<int> solveAssignment(const CostMatrix& cost) {
  if (cost.empty() || cost[0].empty()) return std::vector<int>(cost.size(), -1);
  const size_t rows = cost.size();
  const size_t cols = cost[0].size();
  for (const auto& row : cost) {
    if (row.size() != cols) throw std::invalid_argument("solveAssignment: ragged matrix");
  }

  if (rows <= cols) return hungarianRowsLeqCols(cost);

  // More rows than columns: solve the transpose and invert the answer.
  CostMatrix transposed(cols, std::vector<double>(rows));
  for (size_t r = 0; r < rows; ++r)
    for (size_t c = 0; c < cols; ++c) transposed[c][r] = cost[r][c];
  std::vector<int> col_to_row = hungarianRowsLeqCols(transposed);
  std::vector<int> row_to_col(rows, -1);
  for (size_t c = 0; c < cols; ++c) {
    if (col_to_row[c] >= 0) row_to_col[col_to_row[c]] = static_cast<int>(c);
  }
  return row_to_col;
}

double assignmentCost(const CostMatrix& cost, const std::vector<int>& row_to_col) {
  double total = 0;
  for (size_t r = 0; r < row_to_col.size(); ++r) {
    if (row_to_col[r] >= 0) total += cost[r][row_to_col[r]];
  }
  return total;
}
