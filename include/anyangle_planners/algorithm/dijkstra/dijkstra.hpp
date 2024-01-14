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
#include <unordered_map>  // TODO: to be replaced by better hashmaps

#include "anyangle_planners/algorithm/dijkstra/vertex.hpp"
#include "anyangle_planners/algorithm/planner.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

template <typename StateSpaceType, typename EnvironmentType>
class Dijkstra
  : public PlannerBase<Dijkstra<StateSpaceType, EnvironmentType>, StateSpaceType, EnvironmentType>
{
public:
  Dijkstra(const std::string& name) : PlannerBase<Dijkstra, StateSpaceType, EnvironmentType>(name)
  {
  }

  void reset();

  bool solve(const StateSpaceType& start, const StateSpaceType& goal,
             const EnvironmentType& planning_problem);

protected:
  Vertex* addToGraph(const StateSpaceType& state, const EnvironmentType& problem);

  // custom function for returning minimum distance vertex
  // to be used in priority queue
  struct VertexComparator
  {
    // operator overloading
    bool operator()(const Vertex* v1, const Vertex* v2) const { return *v1 > *v2; }
  };

  using PQueueT = std::priority_queue<Vertex*, std::vector<Vertex*>, VertexComparator>;
  using Graph = std::unordered_map<unsigned int, std::unique_ptr<Vertex>>;

  /// @brief Pointers to start and goal vertices
  Vertex* start_vertex_;
  Vertex* goal_vertex_;

  //// openlist (frontier) for dijkstra algorithm
  PQueueT open_list_;

  //// closelist (expanded vertices)
  std::vector<Vertex*> close_list_;

  //// graph of processed vertices so far
  Graph graph_;
};

}  // namespace algorithm::dijkstra
}  // namespace anyangle

#include "anyangle_planners/algorithm/dijkstra/impl/dijkstra.hpp"