#include "anyangle_planners/map/scenario_loader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace anyangle {
namespace map {
namespace moving_ai_lab {

bool ScenarioLoader::loadScenario(const std::string& path_to_scenario_file,
                                  Scenario& scenario) {
  std::ifstream input_file(path_to_scenario_file);

  if (input_file.fail()) {
    std::cerr << "Failed to open MovingAILab scenario file "
              << path_to_scenario_file << std::endl;
    return false;
  }

  // check version number exists
  std::string temp;
  input_file >> temp;
  if (temp != "version" && temp != "Version") {
    std::cerr << "Scenario File Missing Version Number." << std::endl;
    return false;
  }

  // create scenario
  scenario.name = path_to_scenario_file;
  input_file >> scenario.version;

  scenario.experiments.clear();

  // read scenario file and create Scenario experiments
  std::string line;
  while (std::getline(input_file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);

    Scenario::Experiment experiment;
    ss >> experiment.bucket >> experiment.map_name >> experiment.map_width >>
        experiment.map_height >> experiment.start_x >> experiment.start_y >>
        experiment.goal_x >> experiment.goal_y >> experiment.optimal_length;

    scenario.experiments.emplace_back(experiment);
  }

  return true;
}

}  // namespace moving_ai_lab
}  // namespace map
}  // namespace anyangle