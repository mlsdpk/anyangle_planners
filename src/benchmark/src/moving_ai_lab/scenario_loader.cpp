
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

#include "anyangle_planners/benchmark/moving_ai_lab/scenario_loader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace anyangle::benchmark {
namespace moving_ai_lab {

bool ScenarioLoader::loadScenario(const std::string& path_to_scenario_file, Scenario& scenario)
{
  std::ifstream input_file(path_to_scenario_file);

  if (input_file.fail())
  {
    std::cerr << "Failed to open MovingAILab scenario file " << path_to_scenario_file << std::endl;
    return false;
  }

  // check version number exists
  std::string temp;
  input_file >> temp;
  if (temp != "version" && temp != "Version")
  {
    std::cerr << "Scenario File Missing Version Number." << std::endl;
    return false;
  }

  // create scenario
  scenario.name = path_to_scenario_file;
  input_file >> scenario.version;

  scenario.experiments.clear();

  // read scenario file and create Scenario experiments
  std::string line;
  while (std::getline(input_file, line))
  {
    if (line.empty()) continue;
    std::stringstream ss(line);

    Scenario::Experiment experiment;
    ss >> experiment.bucket >> experiment.map_name >> experiment.map_width >>
        experiment.map_height >> experiment.start_x >> experiment.start_y >> experiment.goal_x >>
        experiment.goal_y >> experiment.optimal_length;

    scenario.experiments.emplace_back(experiment);
  }

  return true;
}

}  // namespace moving_ai_lab
}  // namespace anyangle::benchmark