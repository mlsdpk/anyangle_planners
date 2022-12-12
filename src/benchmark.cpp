#include <iostream>
#include <memory>
#include <string>

#include "anyangle_planners/algorithm/a_star.hpp"
#include "anyangle_planners/algorithm/planner.hpp"
#include "anyangle_planners/map/environment.hpp"
#include "anyangle_planners/map/scenario_loader.hpp"

int main(int argc, char const* argv[]) {
  std::string benchmark_template_file;

  if (argc == 1) {
    printf("Requird to provide template json file name. Exiting...\n");
    exit(0);
  } else if (argc > 2) {
    printf("Error. Multiple arguments given. Exiting...");
  } else {
    benchmark_template_file = argv[1];
  }

  std::cout << "Reading benchmark template file from " << benchmark_template_file << std::endl;

  using namespace anyangle::map;
  moving_ai_lab::ScenarioLoaderPtr loader = std::make_shared<moving_ai_lab::ScenarioLoader>();

  moving_ai_lab::Scenario scenario;
  loader->loadScenario(benchmark_template_file, scenario);

  // create environment from scenario
  auto env = std::make_shared<Environment>();
  env->initializeFromMovingAILabScenario(scenario);

  using namespace anyangle::algorithm;

  // lets create an example planner
  auto planner = std::make_shared<AStar>("astar");

  // provide an environment
  planner->setEnvironment(env);

  // run all the experiments inside scenario
  {
    for (std::size_t i = 0; i < scenario.experiments.size(); ++i) {
      auto start_x = scenario.experiments[i].start_x;
      auto start_y = scenario.experiments[i].start_y;
      auto goal_x = scenario.experiments[i].goal_x;
      auto goal_y = scenario.experiments[i].goal_y;

      // solve
      bool solved = planner->solve(
          anyangle::State2D(static_cast<float>(start_x), static_cast<float>(start_x)),
          anyangle::State2D(static_cast<float>(goal_x), static_cast<float>(goal_y)));

      if (solved) {
        auto bucket = scenario.experiments[i].bucket;

        std::cout << "Experiment " << i << ", Bucket " << bucket << " solved with path length of "
                  << planner->getPathCost() << std::endl;
        std::cout << "Actual optimal path cost is " << scenario.experiments[i].optimal_length
                  << std::endl;
      }

      break;
    }
  }

  return 0;
}