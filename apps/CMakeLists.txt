# Specify the benchmark target
add_executable(benchmark
  benchmark.cpp)

# Specify include directories for this target.
target_include_directories(benchmark
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(benchmark
  PRIVATE
  PUBLIC
    nlohmann_json::nlohmann_json)