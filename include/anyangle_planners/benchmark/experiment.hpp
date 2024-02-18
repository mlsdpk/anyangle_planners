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
  explicit Experiment(const config::Config& config);

  void setup();

  void run(bool verbose = true);

private:
  config::Config config_;

  moving_ai_lab::Scenario moving_ai_lab_scenario_;

  anyangle::algorithm::dijkstra::env_t graph_env_;
};

}  // namespace benchmark
}  // namespace anyangle