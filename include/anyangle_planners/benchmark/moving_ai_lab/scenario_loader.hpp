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

#include <memory>
#include <string>
#include <unordered_map>

#include "anyangle_planners/benchmark/moving_ai_lab/scenario.hpp"

namespace anyangle::benchmark {
namespace moving_ai_lab {

class ScenarioLoader
{
public:
  using ScenarioList = std::unordered_map<std::string, Scenario>;

  ScenarioLoader() = default;

  bool loadScenario(const std::string& path_to_scenario_file, Scenario& scenario);

private:
  ScenarioList scenarios_;
};

typedef std::shared_ptr<ScenarioLoader> ScenarioLoaderPtr;

}  // namespace moving_ai_lab
}  // namespace anyangle::benchmark