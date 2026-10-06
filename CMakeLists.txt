cmake_minimum_required(VERSION 3.14)
project(fleet_dispatch LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(fleet
  src/graph.cpp
  src/city.cpp
  src/demand.cpp
  src/assignment.cpp
  src/simulator.cpp
  src/policies.cpp
  src/metrics.cpp
  src/scenario.cpp
)
target_include_directories(fleet PUBLIC src)

add_executable(dispatch src/main.cpp)
target_link_libraries(dispatch PRIVATE fleet)

add_executable(fleet_tests
  tests/test_main.cpp
  tests/test_graph.cpp
  tests/test_assignment.cpp
  tests/test_simulator.cpp
)
target_link_libraries(fleet_tests PRIVATE fleet)

enable_testing()
add_test(NAME fleet_tests COMMAND fleet_tests)
