#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
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

  using namespace anyangle::map;
  using namespace anyangle::algorithm;

  namespace fs = std::filesystem;

  // find all .scen files inside the directory
  std::string ext(".scen");
  for (auto& p : fs::recursive_directory_iterator(benchmark_template_file)) {
    if (p.path().extension() == ext) {
      std::cout << p.path().string() << '\n';

      moving_ai_lab::ScenarioLoaderPtr loader = std::make_shared<moving_ai_lab::ScenarioLoader>();

      moving_ai_lab::Scenario scenario;
      loader->loadScenario(p.path().string(), scenario);

      // create environment from scenario
      auto env = std::make_shared<Environment>();
      env->initializeFromMovingAILabScenario(scenario);

      // lets create an example planner
      auto planner = std::make_shared<AStar>("astar");

      // provide an environment
      planner->setEnvironment(env);

      std::vector<PlannerPtr> planners;
      planners.push_back(planner);

      // run all the experiments inside scenario for each planner
      for (std::size_t planner_id = 0; planner_id < planners.size(); ++planner_id) {
        // create file to store the data
        // std::string output_folder = "output";
        // std::string output_file = scenario.experiments[0].map_name + std::string(".txt");
        // fs::create_directory(output_folder);
        // std::ofstream out(output_folder + "/" + output_file);

        std::vector<double> path_costs;
        std::vector<unsigned> run_times;

        for (std::size_t i = 0; i < scenario.experiments.size(); ++i) {
          auto start_x = scenario.experiments[i].start_x;
          auto start_y = scenario.experiments[i].start_y;
          auto goal_x = scenario.experiments[i].goal_x;
          auto goal_y = scenario.experiments[i].goal_y;

          // out << "Experiment No." << i << " : start_x: " << start_x << ", start_y: " << start_y
          //     << ", goal_x: " << goal_x << ", goal_y: " << goal_y << std::endl;

          // solve
          auto start_time = std::chrono::high_resolution_clock::now();
          bool solved = planners[planner_id]->solve(
              anyangle::State2D(static_cast<float>(start_x), static_cast<float>(start_y)),
              anyangle::State2D(static_cast<float>(goal_x), static_cast<float>(goal_y)));
          auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::high_resolution_clock::now() - start_time)
                                     .count();

          if (solved) {
            auto bucket = scenario.experiments[i].bucket;

            path_costs.push_back(planners[planner_id]->getPathCost());
            run_times.push_back(elapsed_time_ms);

            // std::cout << "Experiment " << i << ", Bucket " << bucket << " solved with path length
            // of
            // "
            //           << planner->getPathCost() << std::endl;
            // std::cout << "Actual optimal path cost is " << scenario.experiments[i].optimal_length
            //           << std::endl;
            // std::cout << "Computation takes " << elapsed_time_ms << " [ms].\n";

            // {
            //   out << std::to_string(i) << " " << std::to_string(bucket) << " "
            //       << std::to_string(planners[planner_id]->getPathCost()) << " "
            //       << std::to_string(elapsed_time_ms) << std::endl;
            // }

            // get the solution path
            anyangle::State2DList path;
            planners[planner_id]->getSolutionPath(path);

            // auto img = env->toImage(path);
            // cv::imwrite("test.jpg", img);
          }
        }

        // find the average metrics
        auto avg_path_cost = std::reduce(path_costs.begin(), path_costs.end()) / path_costs.size();
        auto avg_run_time =
            std::reduce(run_times.begin(), run_times.end()) / run_times.size() * 1.0;

        std::cout << "Avg path cost: " << avg_path_cost << ", Avg run time: " << avg_run_time
                  << std::endl;

        return 0;
      }
    }
  }

  return 0;
}