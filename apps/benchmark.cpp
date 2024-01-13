// MIT License

// Copyright (c) 2023 Phone Thiha Kyaw

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>

#include "anyangle_planners/algorithm/dijkstra/dijkstra.hpp"
#include "anyangle_planners/benchmark/config.hpp"
#include "anyangle_planners/benchmark/experiment.hpp"
#include "anyangle_planners/graph/point2d.hpp"
#include "anyangle_planners/registration.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

// namespace anyangle {

// struct BenchmarkConfig
// {
//   std::vector<PlannerID> planners;
//   EnvironmentID environment_id;
// };

// void from_json(const json& j, BenchmarkConfig& config)
// {
//   // update planners
//   if (j.contains("planners"))
//   {
//     auto j_planners = j["planners"];
//     for (const auto& j_id : j_planners)
//     {
//       config.planners.push_back(j_id.template get<anyangle::PlannerID>());
//     }
//   }

//   // update environment type
//   if (j.contains("environment"))
//   {
//     config.environment_id = j["environment"].template get<anyangle::EnvironmentID>();
//   }
// }

// }  // namespace anyangle

int main(int argc, char const* argv[])
{
  std::string benchmark_template_file;

  if (argc == 1)
  {
    printf("Requird to provide template json file name. Exiting...\n");
    exit(0);
  }
  else if (argc > 2)
  {
    printf("Error. Multiple arguments given. Exiting...\n");
  }
  else
  {
    benchmark_template_file = argv[1];
  }

  std::ifstream f(benchmark_template_file);
  json data = json::parse(f);

  if (data.empty() || !data.contains("experiment"))
  {
    printf("Error. json key 'experiment' not found. Exiting...\n");
    exit(0);
  }

  // from json => benchmark config
  auto config = data["experiment"].template get<anyangle::benchmark::config::Config>();
  config.config_file_path = benchmark_template_file;

  // create experiment
  auto exp = anyangle::benchmark::Experiment(config);

  // run experiment
  // exp->run(/* verbose = */ true);

  // testing
  {
    using state_space_t = anyangle::graph::Point2D<double>;

    anyangle::algorithm::Dijkstra<state_space_t> dijkstra_algorithm{"dijkstra"};
    dijkstra_algorithm.reset();

    auto start_state = anyangle::graph::Point2D(0.0, 0.0);
    auto goal_state = anyangle::graph::Point2D(1.0, 1.0);

    dijkstra_algorithm.solve(start_state, goal_state);
  }

  return 0;
}