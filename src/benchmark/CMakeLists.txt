add_library(anyangle_planners_benchmark
    src/config.cpp
    src/moving_ai_lab/scenario_loader.cpp)

target_include_directories(anyangle_planners_benchmark
    PUBLIC
        ${PROJECT_SOURCE_DIR}/thirdparty
        ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(anyangle_planners_benchmark
  PRIVATE
  PUBLIC
    nlohmann_json::nlohmann_json
    tabulate::tabulate)