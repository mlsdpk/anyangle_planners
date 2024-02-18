// MIT License

// Copyright (c) 2024 Phone Thiha Kyaw

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

#pragma once

#include "anyangle_planners/registration.hpp"

namespace anyangle {
namespace benchmark {
namespace config {

////////////////////////////////////////////////////////////////////////////////////

namespace env_type {
struct Base
{
  /// @brief environment type representation (INVALID = 0, GRIDMAP = 1)
  EnvironmentType env_type;

  /// @brief Distance metric to use during search
  /// INVALID = 0, EUCLIDEAN = 1, MANHATTAN = 2, OCTILE = 3
  DistanceMetric distance_metric;
};

struct GridMap : Base
{
  /// @brief Type of grid connectivity
  /// FOUR  = 0 (four-connected grid)
  /// EIGHT = 1 (eight-connected grid)
  grid_map::Connectivity connectivity;
};
}  // namespace env_type

namespace env_id {
struct Base
{
  virtual ~Base() = default;

  /// @brief registered environment IDs (INVALID = 0, MovingAILabScenario = 1)
  EnvironmentID env_id;
};

struct MovingAILabScenario : public Base
{
  /// @brief Name of the moving AI Lab scenario file
  std::string scenario_file_name;
};
}  // namespace env_id

/**
 * @brief Params related to environmental representation of the planning experiment
 *
 */
struct Environment
{
  std::shared_ptr<env_type::Base> env_type_params;
  std::shared_ptr<env_id::Base> env_id_params;
};

////////////////////////////////////////////////////////////////////////////////////

struct Config
{
  /// @brief Path to this benchmark config file
  std::string config_file_path;

  /// @brief list of planners to use during benchmarking
  std::vector<PlannerID> planners;

  /// @brief params related to environmental representation of the planning experiment
  Environment env_params;

  /// @brief Number of experiments to run
  unsigned int num_of_runs;
};

void from_json(const nlohmann::json& j, Config& config);

}  // namespace config
}  // namespace benchmark
}  // namespace anyangle