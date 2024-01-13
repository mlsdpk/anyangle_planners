#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "anyangle_planners/map/scenario.hpp"

namespace anyangle {
namespace map {
namespace moving_ai_lab {

class ScenarioLoader {
 public:
  using ScenarioList = std::unordered_map<std::string, Scenario>;

  ScenarioLoader() = default;

  bool loadScenario(const std::string& path_to_scenario_file,
                    Scenario& scenario);

 private:
  ScenarioList scenarios_;
};

typedef std::shared_ptr<ScenarioLoader> ScenarioLoaderPtr;

}  // namespace moving_ai_lab
}  // namespace map
}  // namespace anyangle