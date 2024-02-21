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

#include "anyangle_planners/anyangle_planners.hpp"
#include "anyangle_planners/benchmark/config.hpp"
#include "anyangle_planners/benchmark/environment_loader.hpp"
#include "anyangle_planners/benchmark/moving_ai_lab/scenario_loader.hpp"
#include "tabulate/table.hpp"

namespace anyangle {
namespace benchmark {

class Experiment
{
public:
  explicit Experiment(const config::Config& config) : config_{config}
  {
    using namespace tabulate;

    Table header_table;

    header_table.format().border_color(Color::yellow);

    header_table.add_row(
        Table::Row_t{"Algorithms for Any-angle Pathfinding written in Modern C++"});
    header_table[0].format().font_align(FontAlign::center).font_color(Color::yellow);

    header_table.add_row(Table::Row_t{"https://github.com/mlsdpk/anyangle_planners"});
    header_table[1]
        .format()
        .font_align(FontAlign::center)
        .font_style({FontStyle::underline, FontStyle::italic})
        .font_color(Color::white)
        .hide_border_top();

    Table empty_row1;
    empty_row1.format().hide_border();
    header_table.add_row(Table::Row_t{empty_row1});

    header_table.add_row(
        Table::Row_t{"Using benchmark template json file from " + config.config_file_path});
    header_table[3].format().font_color(Color::magenta);
    header_table[3].format().hide_border();

    // Print the table
    std::cout << header_table << "\n";

    Table configs_table;
    configs_table.add_row(
        Table::Row_t{"Type", toString(config.env_params.env_type_params->env_type)});
    configs_table.add_row(Table::Row_t{"ID", toString(config.env_params.env_id_params->env_id)});
    configs_table.add_row(Table::Row_t{"No: of runs", std::to_string(config.num_of_runs)});

    configs_table.add_row(Table::Row_t{"Planners", [&]() {
                                         std::string planner_list_str;
                                         for (const auto id : config.planners)
                                         {
                                           planner_list_str += toString(id);
                                           planner_list_str += std::string("\n");
                                         }

                                         return planner_list_str;
                                       }()});

    configs_table[0].format().font_align(FontAlign::center);
    configs_table[1].format().font_align(FontAlign::center);
    configs_table[2].format().font_align(FontAlign::center);
    configs_table[3].format().font_align(FontAlign::center);

    configs_table.column(0)
        .format()
        .font_align(FontAlign::right)
        .font_color(Color::green)
        .font_background_color(Color::grey);

    configs_table.column(1).format().font_align(FontAlign::left);

    std::cout << configs_table << "\n\n";
  }

  void run(bool verbose = true)
  {
    auto env_id = config_.env_params.env_id_params->env_id;

    if (env_id == anyangle::EnvironmentID::MovingAILabScenario)
    {
      auto env_id_params = std::dynamic_pointer_cast<config::env_id::MovingAILabScenario>(
          config_.env_params.env_id_params);

      // create moving AI Lab scenario loader
      auto scenario_loader = std::make_unique<moving_ai_lab::ScenarioLoader>();

      if (!scenario_loader->loadScenario(env_id_params->scenario_file_name,
                                         moving_ai_lab_scenario_))
      {
        exit(0);
      }

      // create graph environment from AI Lab scenario
      // TODO: handle environment based on configs
      auto graph_env = create(moving_ai_lab_scenario_);

      // TODO: run for all experiments
      // TODO: handle based on configs

      if (moving_ai_lab_scenario_.experiments.empty()) return;

      const auto width = moving_ai_lab_scenario_.experiments[1].map_width;

      // width - x, height - y, top-left - (0, 0)
      const auto start_x = moving_ai_lab_scenario_.experiments[0].start_y;
      const auto start_y = moving_ai_lab_scenario_.experiments[0].start_x;
      const auto goal_x = moving_ai_lab_scenario_.experiments[0].goal_y;
      const auto goal_y = moving_ai_lab_scenario_.experiments[0].goal_x;

      const auto start_id = (start_x * width) + start_y;
      const auto goal_id = (goal_x * width) + goal_y;

      using graph_env_t = anyangle::benchmark::graph_t;
      using state_space_t = typename graph_env_t::state_space_t;
      graph_env.setStartAndGoalState(state_space_t(start_id), state_space_t(goal_id));

      using namespace anyangle::algorithm::dijkstra;
      Dijkstra<graph_env_t> dijkstra_algorithm{"dijkstra"};
      dijkstra_algorithm.reset(graph_env);

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      auto solution =
          dijkstra_algorithm.solve(state_space_t(start_id), state_space_t(goal_id), graph_env);
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      if (solution.has_value())
      {
        std::cout << "Solution cost: " << solution.value().second << std::endl;
      }
      else
      {
        std::cout << "Failed to find a solution\n";
      }

      std::cout << "Computation time = "
                << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
                << "[us]" << std::endl;
    }
  }

private:
  config::Config config_;

  moving_ai_lab::Scenario moving_ai_lab_scenario_;

  anyangle::algorithm::dijkstra::env_t graph_env_;
};

}  // namespace benchmark
}  // namespace anyangle