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

#include <cmath>
#include <fstream>
#include <iostream>

#include "anyangle_planners/anyangle_planners.hpp"
#include "anyangle_planners/benchmark/moving_ai_lab/scenario.hpp"
#include "anyangle_planners/environment.hpp"

namespace anyangle::benchmark {

/// @brief Temporarily put here for now
enum class Connectivity
{
  FOUR,  // four-connected grid
  EIGHT  // eight-connected grid
};

template <typename StateSpaceType,
          anyangle::utils::DistanceType DistanceMetric = anyangle::utils::DistanceType::EUCLIDEAN>
class DistanceCost
  : public anyangle::environment::CostBase<DistanceCost<StateSpaceType, DistanceMetric>,
                                           StateSpaceType>
{
public:
  using state_space_t = StateSpaceType;

  static decltype(auto) cost(const state_space_t& from, const state_space_t& to)
  {
    auto indexTo2D = [&](int i) {
      const auto ix = i / width;
      const auto iy = i % width;
      return std::make_pair(ix, iy);
    };

    if constexpr (DistanceMetric == anyangle::utils::DistanceType::MANHATTAN)
    {
      auto [from_x, from_y] = indexTo2D(from[0]);
      auto [to_x, to_y] = indexTo2D(to[0]);
      return anyangle::utils::manhattanDistance(from_x, from_y, to_x, to_y);
    }
    else if constexpr (DistanceMetric == anyangle::utils::DistanceType::EUCLIDEAN)
    {
      auto [from_x, from_y] = indexTo2D(from[0]);
      auto [to_x, to_y] = indexTo2D(to[0]);
      return anyangle::utils::euclideanDistance(from_x, from_y, to_x, to_y);
    }
    else if constexpr (DistanceMetric == anyangle::utils::DistanceType::OCTILE)
    {
      auto [from_x, from_y] = indexTo2D(from[0]);
      auto [to_x, to_y] = indexTo2D(to[0]);
      return anyangle::utils::octileDistance(from_x, from_y, to_x, to_y);
    }
  }

  static int width;
};

template <typename StateSpaceType, anyangle::utils::DistanceType DistanceMetric>
int DistanceCost<StateSpaceType, DistanceMetric>::width;

using state_space_t = anyangle::environment::graph::VertexStateSpace;
using vertex_t = anyangle::algorithm::dijkstra::vertex_t;
using vertex_id_t = anyangle::environment::graph::vertex_id_t;
using edge_t = anyangle::environment::graph::EdgePropertiesBase<void>;
using cost_t = DistanceCost<state_space_t, anyangle::utils::DistanceType::OCTILE>;
using graph_t = anyangle::environment::graph::Graph<cost_t, state_space_t, vertex_t, edge_t>;

/**
 * @brief Convenient method to create environment::graph::Graph environment
 * from moving AI Lab scenario file.
 *
 * TODO: check valid GraphT is provided
 *
 * @param scenario
 * @return environment::graph::Graph
 */
inline decltype(auto) create(const moving_ai_lab::Scenario& scenario,
                             Connectivity connectivity = Connectivity::EIGHT)
{
  // check the scenario version is supported or not
  if (scenario.version != "1")
  {
    std::cerr << "Only version 1 is currently supported for Moving AI Lab scenario.\n";
    exit(0);
  }

  // must contain at least one experiment
  if (scenario.experiments.empty())
  {
    std::cerr << "Scenario must contain at least one experiment.\n";
    exit(0);
  }

  // here we assume that all the experiments inside scenario share the same map name
  // so we only use the first experiment map name
  std::string map_name = scenario.experiments.front().map_name;

  std::ifstream map_file(map_name);
  if (map_file.fail() && scenario.name.find('/') != std::string::npos)
  {
    map_name = scenario.name.substr(0, scenario.name.find_last_of('/')) + "/" + map_name;
  }

  std::cout << "Loading Moving AI Lab scenario map from " << map_name << std::endl;
  map_file = std::ifstream(map_name);

  if (map_file.fail())
  {
    std::cerr << "Failed to load map file from " << map_name << std::endl;
    exit(0);
  }

  std::string temp;
  std::getline(map_file, temp);

  int height;
  int width;

  map_file >> temp >> height;
  map_file >> temp >> width;
  map_file >> temp;
  std::getline(map_file, temp);

  const auto vertex_count = width * height;

  // 1D flatten temporary array to store collision information
  std::vector<bool> temp_map;
  temp_map.resize(width * height);

  size_t row_idx = 0u;
  for (std::string line; std::getline(map_file, line);)
  {
    size_t col_idx = 0u;
    for (std::string::iterator it = line.begin(); it != line.end(); ++it)
    {
      // compute 1D index from 2D coordinate
      const auto index = (row_idx * width) + col_idx;

      if ((*it) == '.')
      {
        temp_map[index] = true;
      }
      else if ((*it) == '@')
      {
        temp_map[index] = false;
      }

      ++col_idx;
    }
    ++row_idx;
  }

  std::vector<int> neighbors_grid_offsets;
  // 4-connectivity
  if (connectivity == Connectivity::FOUR)
  {
    neighbors_grid_offsets = {-1, +1, -width, +width};
  }
  // 8-connectivity
  else
  {
    neighbors_grid_offsets = {-1,         +1,         -width,     +width,
                              -width - 1, -width + 1, +width - 1, +width + 1};
  }

  // 1D index to 2D position
  auto indexTo2D = [](int i, int width) {
    const auto ix = i / width;
    const auto iy = i % width;
    return std::make_pair(ix, iy);
  };

  graph_t graph{static_cast<size_t>(vertex_count)};
  graph_t::cost_t::width = width;

  for (size_t i = 0; i < temp_map.size(); ++i)
  {
    // add a vertex
    graph.addVertex(i, {});

    // add all neighbors that are traversable from current vertex
    for (const auto offset : neighbors_grid_offsets)
    {
      const auto neighbor_index = i + offset;

      const auto ix = i / width;
      const auto iy = i % width;
      const auto neighbor_x = neighbor_index / width;
      const auto neighbor_y = neighbor_index % width;

      // convert both back to 2D
      // check if this is valid neighbor (here wraping is not allowed)
      if ((std::max(ix, neighbor_x) - std::min(ix, neighbor_x)) > 1 ||
          (std::max(iy, neighbor_y) - std::min(iy, neighbor_y)) > 1)
        continue;

      // check if the neighbor vertex is collision-free
      if (!temp_map[neighbor_index]) continue;

      // check if the neighbor vertex is within map bounds
      if (neighbor_index < 0 || neighbor_index >= (width * height)) continue;

      // TODO(Phone): compute edge weight based on distance metric
      graph.addEdge(
          i, neighbor_index,
          {graph.cost(typename graph_t::state_space_t{static_cast<vertex_id_t>(i)},
                      typename graph_t::state_space_t{static_cast<vertex_id_t>(neighbor_index)})});
    }
  }

  return graph;
}

}  // namespace anyangle::benchmark