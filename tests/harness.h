#pragma once

// A deliberately tiny test harness: TEST(name) registers a function, CHECK
// records a failure without stopping the test, and test_main.cpp runs them all.

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

struct TestCase {
  std::string name;
  std::function<void()> body;
};

inline std::vector<TestCase>& testRegistry() {
  static std::vector<TestCase> tests;
  return tests;
}

inline int& checkFailures() {
  static int failures = 0;
  return failures;
}

struct TestRegistrar {
  TestRegistrar(const char* name, std::function<void()> body) { testRegistry().push_back({name, std::move(body)}); }
};

#define TEST(name)                                       \
  static void name();                                    \
  static TestRegistrar registrar_##name(#name, name);    \
  static void name()

#define CHECK(condition)                                                                    \
  do {                                                                                      \
    if (!(condition)) {                                                                     \
      ++checkFailures();                                                                    \
      std::cerr << "  " << __FILE__ << ":" << __LINE__ << ": CHECK(" #condition ") failed\n"; \
    }                                                                                       \
  } while (0)

#define CHECK_NEAR(a, b, tolerance)                                                               \
  do {                                                                                            \
    double check_a_ = (a), check_b_ = (b);                                                        \
    if (!(std::fabs(check_a_ - check_b_) <= (tolerance))) {                                       \
      ++checkFailures();                                                                          \
      std::cerr << "  " << __FILE__ << ":" << __LINE__ << ": CHECK_NEAR(" #a ", " #b ") failed: " \
                << check_a_ << " vs " << check_b_ << "\n";                                        \
    }                                                                                             \
  } while (0)
