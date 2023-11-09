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

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "nlohmann/json.hpp"
#pragma GCC diagnostic pop

namespace anyangle {

/**
 * @brief All the registered planner IDs
 *
 */
enum class PlannerID
{
  INVALID = 0,
  DIJKSTRA,
  ASTAR,
  THETASTAR
};

/**
 * @brief All the registered environment types
 *
 */
enum class EnvironmentID
{
  INVALID = 0,
  MovingAILabScenario
};

NLOHMANN_JSON_SERIALIZE_ENUM(PlannerID, {{PlannerID::INVALID, "invalid"},
                                         {PlannerID::DIJKSTRA, "Dijkstra"},
                                         {PlannerID::ASTAR, "AStar"},
                                         {PlannerID::THETASTAR, "ThetaStar"}})

NLOHMANN_JSON_SERIALIZE_ENUM(EnvironmentID,
                             {{EnvironmentID::INVALID, "invalid"},
                              {EnvironmentID::MovingAILabScenario, "MovingAILabScenario"}})

}  // namespace anyangle