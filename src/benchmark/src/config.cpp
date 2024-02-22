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

#include "anyangle_planners/benchmark/config.hpp"

#include <iostream>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace anyangle {
namespace benchmark {
namespace config {

void from_json(const json& j, Config& config)
{
  // update planners
  if (j.contains("planners"))
  {
    auto j_planners = j["planners"];
    for (const auto& j_id : j_planners)
    {
      config.planners.push_back(j_id.template get<anyangle::PlannerID>());
    }
  }

  // update environment config
  if (j.contains("environment"))
  {
    // get environment type and id
    auto env_type = j["environment"]["type"].template get<anyangle::EnvironmentType>();
    auto env_id = j["environment"]["id"].template get<anyangle::EnvironmentID>();

    if (env_type == anyangle::EnvironmentType::GRIDMAP)
    {
      config.env_params.env_type_params = std::make_shared<env_type::GridMap>();
      config.env_params.env_type_params->env_type = env_type;
    }

    if (env_id == anyangle::EnvironmentID::MovingAILabScenario)
    {
      config.env_params.env_id_params = std::make_shared<env_id::MovingAILabScenario>();

      auto derived =
          std::dynamic_pointer_cast<env_id::MovingAILabScenario>(config.env_params.env_id_params);

      derived->env_id = env_id;

      auto scenario_file_name = j["environment"]["MovingAILabScenario"];
      derived->scenario_file_name = scenario_file_name.value("scenario_file_name", "");
    }
  }

  if (j.contains("num_of_runs"))
  {
    config.num_of_runs = j["num_of_runs"].template get<unsigned>();
  }
}

}  // namespace config
}  // namespace benchmark
}  // namespace anyangle