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

#include "anyangle_planners/algorithm/dijkstra/dijkstra.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

template <typename StateSpaceType, typename EnvironmentType>
void Dijkstra<StateSpaceType, EnvironmentType>::reset()
{
  std::cout << "Dijkstra reset has been called\n";
}

template <typename StateSpaceType, typename EnvironmentType>
Vertex* Dijkstra<StateSpaceType, EnvironmentType>::addToGraph(const StateSpaceType& state,
                                                              const EnvironmentType& problem)
{
  // compute graph index from the state input and planning problem
  auto index = toIndex(state[0], state[1], problem.getWidth());
  return graph_.emplace(index, std::make_unique<Vertex>(state[0], state[1], index))
      .first->second.get();
}

template <typename StateSpaceType, typename EnvironmentType>
bool Dijkstra<StateSpaceType, EnvironmentType>::solve(const StateSpaceType& start,
                                                      const StateSpaceType& goal,
                                                      const EnvironmentType& planning_problem)
{
  std::cout << "Solving using Dijkstra algorithm\n";
  std::cout << "Start state: " << start << "\n";
  std::cout << "Goal state: " << goal << "\n";

  // reset intenal stuffs
  reset();

  // add start and goal vertices into graph
  start_vertex_ = addToGraph(start, planning_problem);
  goal_vertex_ = addToGraph(goal, planning_problem);

  bool solved = false;

  return solved;
}

}  // namespace algorithm::dijkstra
}  // namespace anyangle