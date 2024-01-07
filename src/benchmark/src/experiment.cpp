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

#include "anyangle_planners/benchmark/experiment.hpp"

namespace anyangle {
namespace benchmark {

Experiment::Experiment(const config::Config& config)
{
  using namespace tabulate;

  Table header_table;

  header_table.format().border_color(Color::yellow);

  header_table.add_row(Table::Row_t{"Algorithms for Any-angle Pathfinding written in Modern C++"});
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
  configs_table.add_row(Table::Row_t{"Type", toString(config.env_params.env_type_params->env_type)});
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

}  // namespace benchmark
}  // namespace anyangle