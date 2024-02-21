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

#include <iostream>
#include <string>

#include "anyangle_planners/impl/environment/environment.hpp"
#include "anyangle_planners/impl/state_space/state_space.hpp"
#include "anyangle_planners/impl/traits.hpp"

namespace anyangle {
namespace algorithm {

template <typename StateSpaceType>
using solution_t = std::pair<std::vector<StateSpaceType>, double>;

/**
 * @brief Base class to represent a generic any-angle planning algorithm.
 * Note that this class is based on the CRTP design pattern.
 *
 * @tparam Derived Type of the planning algorithm (Dijkstra, A* etc)
 * @tparam StateSpaceType Type of the statespace
 * @tparam EnvironmentType Type of the planning problem
 */
template <typename Derived, typename EnvironmentType,
          typename = internal::traits::IsEnvironment<EnvironmentType>>
class PlannerBase
{
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

public:
  using state_space_t = typename EnvironmentType::state_space_t;

  PlannerBase(const std::string& name) : name_{name} {}

  void reset(EnvironmentType& planning_problem) { derived().reset(); }

  std::optional<solution_t<state_space_t>> solve(const state_space_t& start,
                                                 const state_space_t& goal,
                                                 EnvironmentType& planning_problem)
  {
    return derived().solve(start, goal, planning_problem);
  }

protected:
  /// @brief The name of the planner
  std::string name_;
};

}  // namespace algorithm
}  // namespace anyangle