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

/**
 * @brief Internal environment type representation
 *
 */
enum class EnvironmentType
{
  INVALID = 0,
  GRIDMAP = 1
};

namespace grid_map {
enum class Connectivity
{
  FOUR,  // four-connected grid
  EIGHT  // eight-connected grid
};
}

/**
 * @brief Distance metric to use during planning
 *
 */
enum class DistanceMetric
{
  INVALID = 0,
  EUCLIDEAN = 1,
  MANHATTAN = 2,
  OCTILE = 3
};

NLOHMANN_JSON_SERIALIZE_ENUM(PlannerID, {{PlannerID::INVALID, "invalid"},
                                         {PlannerID::DIJKSTRA, "Dijkstra"},
                                         {PlannerID::ASTAR, "AStar"},
                                         {PlannerID::THETASTAR, "ThetaStar"}})

NLOHMANN_JSON_SERIALIZE_ENUM(EnvironmentID,
                             {{EnvironmentID::INVALID, "invalid"},
                              {EnvironmentID::MovingAILabScenario, "MovingAILabScenario"}})

NLOHMANN_JSON_SERIALIZE_ENUM(EnvironmentType, {{EnvironmentType::INVALID, "invalid"},
                                               {EnvironmentType::GRIDMAP, "GridMap"}})

NLOHMANN_JSON_SERIALIZE_ENUM(grid_map::Connectivity, {{grid_map::Connectivity::FOUR, "Four"},
                                                      {grid_map::Connectivity::EIGHT, "Eight"}})

NLOHMANN_JSON_SERIALIZE_ENUM(DistanceMetric, {{DistanceMetric::INVALID, "invalid"},
                                              {DistanceMetric::EUCLIDEAN, "Euclidean"},
                                              {DistanceMetric::MANHATTAN, "Manhattan"},
                                              {DistanceMetric::OCTILE, "Octile"}})

inline const std::string toString(PlannerID planner_id)
{
  switch (planner_id)
  {
    case PlannerID::DIJKSTRA:
      return "Dijkstra";
    case PlannerID::ASTAR:
      return "AStar";
    case PlannerID::THETASTAR:
      return "ThetaStar";

    default:
      break;
  }

  return "invalid";
}

inline const std::string toString(EnvironmentType env_type)
{
  switch (env_type)
  {
    case EnvironmentType::GRIDMAP:
      return "GridMap";

    default:
      break;
  }

  return "invalid";
}

inline const std::string toString(EnvironmentID env_id)
{
  switch (env_id)
  {
    case EnvironmentID::MovingAILabScenario:
      return "MovingAILabScenario";

    default:
      break;
  }

  return "invalid";
}

}  // namespace anyangle