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

#include <queue>

#include "anyangle_planners/data_structures.hpp"
#include "anyangle_planners/impl/algorithm/planner.hpp"
#include "anyangle_planners/impl/environment/graph.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

struct Vertex : environment::graph::VertexPropertiesBase<Vertex>
{
  double g_value{std::numeric_limits<double>::infinity()};
  double h_value{};
};

using vertex_id_t = environment::graph::vertex_id_t;
using vertex_t = Vertex;
using edge_t = environment::graph::EdgePropertiesBase<void>;
using state_space_t = environment::graph::VertexStateSpace;

class NullCost : public anyangle::environment::CostBase<NullCost, state_space_t>
{
public:
  static decltype(auto) cost(const state_space_t& from, const state_space_t& to) { return 0.0; }
};

using env_t = environment::graph::Graph<NullCost, state_space_t, vertex_t, edge_t>;

namespace internal {
template <typename T>
struct dijkstra_env_type_impl
{
  using type = T;
};

template <>
struct dijkstra_env_type_impl<void>
{
  using type = env_t;
};
}  // namespace internal

template <typename T>
using dijkstra_env_t = typename internal::dijkstra_env_type_impl<T>::type;

template <typename EnvironmentType = void>
class Dijkstra
  : public PlannerBase<Dijkstra<dijkstra_env_t<EnvironmentType>>, dijkstra_env_t<EnvironmentType>>
{
public:
  using env_t = dijkstra_env_t<EnvironmentType>;

  Dijkstra(const std::string& name) : PlannerBase<Dijkstra<env_t>, env_t>(name) {}

  void reset(env_t& planning_problem);

  std::optional<solution_t<state_space_t>> solve(const state_space_t& start,
                                                 const state_space_t& goal,
                                                 env_t& planning_problem);

protected:
  /////////////////////////////////////////////////////////////////////////////////////////

  /// @brief Key to store in the priority queue <f-value, g-value> (priority)
  using open_list_key_t = std::pair<double, double>;

  /// @brief Index to the container storing vertices (value)
  using open_list_value_t = vertex_id_t;

  using open_list_t =
      anyangle::data_structures::priority_queue<8, open_list_key_t, open_list_value_t>;

  /////////////////////////////////////////////////////////////////////////////////////////

  /// @brief IDs to start and goal vertices
  vertex_id_t start_vertex_id_;
  vertex_id_t goal_vertex_id_;

  /// @brief openlist (frontier) for dijkstra algorithm
  open_list_t open_list_;

  /// @brief closelist (expanded vertices)
  std::vector<vertex_id_t> close_list_;

  /// @brief
  std::vector<bool> visited_list_;

  /// @brief Planner configuration parameters
  bool use_astar_heuristics_{false};
};

template <typename EnvironmentType>
void Dijkstra<EnvironmentType>::reset(env_t& planning_problem)
{
  // we will be allocating enough memory for all our variables
  // this is necessary to save expensive memory allocation time
  // during runtime i.e., call to solve().
  // Subsequent calls to solve() will not reserve memory again
  // if the planning problem is never changed

  const auto n = planning_problem.graphOrder();

  visited_list_.assign(n, false);
  close_list_.reserve(n);

  // get start and goal vertex IDs
  auto [start_vertex_id, goal_vertex_id] = planning_problem.getStartAndGoalState();

  start_vertex_id_ = start_vertex_id;
  goal_vertex_id_ = goal_vertex_id;

  // if astar heuristics is used, we precompute h-values (cost-to-go heuristic)
  if (use_astar_heuristics_)
  {
    for (size_t i = 0; i < n; ++i)
    {
      auto& q = planning_problem.vertex(i);
      q.h_value = planning_problem.cost(state_space_t{static_cast<vertex_id_t>(i)},
                                        state_space_t{static_cast<vertex_id_t>(goal_vertex_id_)});
    }
  }

  // get start vertex
  auto& start_vertex = planning_problem.vertex(start_vertex_id_);

  // update start vertex properties
  start_vertex.g_value = 0.0;
  start_vertex.h_value =
      use_astar_heuristics_
          ? planning_problem.cost(state_space_t{static_cast<vertex_id_t>(start_vertex_id_)},
                                  state_space_t{static_cast<vertex_id_t>(goal_vertex_id_)})
          : 0.0;

  // add this start vertex to frontier
  open_list_.push(std::make_pair(start_vertex.g_value + start_vertex.h_value, start_vertex.g_value),
                  start_vertex_id_);
}

template <typename EnvironmentType>
std::optional<solution_t<state_space_t>> Dijkstra<EnvironmentType>::solve(
    const state_space_t& start, const state_space_t& goal, env_t& planning_problem)
{
  bool solved = false;

  // some optimizations for cache
  bool use_astar_heuristics = use_astar_heuristics_;
  auto start_vertex_id = start_vertex_id_;
  auto goal_vertex_id = goal_vertex_id_;

  while (!open_list_.empty())
  {
    // get the index to the minimum vertex for expansion
    const auto [_key, _v_min_id] = open_list_.top();
    const auto v_min_id = _v_min_id;

    // mark this minimum vertex as visited
    visited_list_[v_min_id] = true;

    // remove this vertex from openlist
    open_list_.pop();

    // add this vertex to expanded close list
    close_list_.push_back(v_min_id);

    // if this expanded vertex is goal, solved
    if (v_min_id == goal_vertex_id)
    {
      solved = true;
      break;
    }

    // now we expand more to its neighbors
    // get the minimum vertex from the graph
    const auto v_min_g_value = planning_problem.vertex(v_min_id).g_value;

    planning_problem.forEachNeighbors(
        v_min_id, [&](const vertex_id_t neighbor, const edge_t& edge) mutable {
          // if this neighbor is already visted, do nothing
          if (visited_list_[neighbor]) return;

          // compute new g-value of this neighbor
          const double neighbor_g_value = v_min_g_value + edge.weight;

          // std::cout << "neighbor_g_value: " << neighbor_g_value << std::endl;

          // get neighbor vertex
          auto& neighbor_v = planning_problem.vertex(neighbor);

          // if this new g-value is better than old
          if (neighbor_g_value < neighbor_v.g_value)
          {
            // update the neighbor properties (parent & g-value)
            neighbor_v.parent = v_min_id;
            neighbor_v.g_value = neighbor_g_value;

            double neighbor_h_value = 0.0;
            if (use_astar_heuristics)
            {
              // compute cost-to-go heuristics
              neighbor_h_value = neighbor_v.h_value;
            }

            // add this neighbor to open list for expansion
            open_list_.push(std::make_pair(neighbor_g_value + neighbor_h_value, neighbor_g_value),
                            neighbor);
          }
        });
  }

  solution_t<state_space_t> solution;
  if (solved)
  {
    // if solution found, backtrace to generate a path and compute cost

    auto& path = solution.first;
    auto& cost = solution.second;

    auto current_id = goal_vertex_id;
    while (current_id != start_vertex_id)
    {
      path.emplace_back(state_space_t{static_cast<vertex_id_t>(current_id)});
      auto parent_id = planning_problem.vertex(current_id).parent;
      cost += planning_problem.cost(state_space_t{static_cast<vertex_id_t>(current_id)},
                                    state_space_t{static_cast<vertex_id_t>(parent_id)});
      current_id = parent_id;
    }

    path.emplace_back(state_space_t{static_cast<vertex_id_t>(start_vertex_id)});
    cost += planning_problem.cost(state_space_t{static_cast<vertex_id_t>(current_id)},
                                  state_space_t{static_cast<vertex_id_t>(start_vertex_id)});
  }

  return solved ? std::make_optional(std::move(solution)) : std::nullopt;
}

}  // namespace algorithm::dijkstra
}  // namespace anyangle