// MIT License

// Copyright (c) 2023 Phone Thiha Kyaw

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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>

#include "anyangle_planners/anyangle_planners.hpp"
#include "anyangle_planners/benchmark/config.hpp"
#include "anyangle_planners/benchmark/experiment.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

int main(int argc, char const* argv[])
{
  std::string benchmark_template_file;

  if (argc == 1)
  {
    printf("Requird to provide template json file name. Exiting...\n");
    exit(0);
  }
  else if (argc > 2)
  {
    printf("Error. Multiple arguments given. Exiting...\n");
  }
  else
  {
    benchmark_template_file = argv[1];
  }

  std::ifstream f(benchmark_template_file);
  json data = json::parse(f);

  if (data.empty() || !data.contains("experiment"))
  {
    printf("Error. json key 'experiment' not found. Exiting...\n");
    exit(0);
  }

  // from json => benchmark config
  auto config = data["experiment"].template get<anyangle::benchmark::config::Config>();
  config.config_file_path = benchmark_template_file;

  // create experiment
  auto exp = anyangle::benchmark::Experiment(config);

  // run experiment
  exp.run(/* verbose = */ true);

  return 0;
}