#pragma once

#include <vector>

using CostMatrix = std::vector<std::vector<double>>;

// Cost used for a pair that is not allowed (e.g. the car cannot arrive before
// the rider gives up). It is large enough that the solver prefers any number
// of real pairs over one forbidden pair, so the result maximizes the number of
// allowed matches first and then minimizes their total cost.
constexpr double kForbidden = 1e7;

// Hungarian algorithm (Kuhn-Munkres, O(n^2 m) potentials version). Works on
// any rectangular matrix and matches exactly min(rows, cols) pairs.
// Returns, for each row, the matched column or -1 if the row is unmatched.
std::vector<int> solveAssignment(const CostMatrix& cost);

// Sum of cost[row][col] over matched rows.
double assignmentCost(const CostMatrix& cost, const std::vector<int>& row_to_col);
