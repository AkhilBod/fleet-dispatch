#include <exception>
#include <iostream>
#include <string>

#include "harness.h"

// Usage: fleet_tests [substring]  runs every test whose name contains substring.
int main(int argc, char** argv) {
  std::string filter = argc > 1 ? argv[1] : "";
  int run = 0, failed = 0;
  for (const TestCase& test : testRegistry()) {
    if (test.name.find(filter) == std::string::npos) continue;
    ++run;
    int before = checkFailures();
    bool threw = false;
    try {
      test.body();
    } catch (const std::exception& e) {
      threw = true;
      std::cerr << "  exception: " << e.what() << "\n";
    }
    bool ok = !threw && checkFailures() == before;
    if (!ok) ++failed;
    std::cout << (ok ? "[ PASS ] " : "[ FAIL ] ") << test.name << "\n";
  }
  std::cout << "\n" << (run - failed) << "/" << run << " tests passed\n";
  return failed == 0 ? 0 : 1;
}
