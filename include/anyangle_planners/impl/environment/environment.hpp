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

#include "anyangle_planners/impl/traits.hpp"
#include "anyangle_planners/impl/utils/distance.hpp"

namespace anyangle {
namespace environment {

template <typename Derived, typename StateSpaceType>
class CostBase
{
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

public:
  static decltype(auto) cost(const StateSpaceType& from, const StateSpaceType& to)
  {
    return Derived::cost(from, to);
  }
};

template <typename Derived, typename StateSpaceType, typename CostType,
          typename = anyangle::internal::traits::IsStateSpace<StateSpaceType>>
class EnvironmentBase
{
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

public:
  using state_space_t = StateSpaceType;
  using cost_t = CostType;

  bool inCollision(internal::traits::pass_type<StateSpaceType> state) const
  {
    return derived().isInCollision(state);
  }

  decltype(auto) getStartAndGoalState() const { return derived().getStartAndGoalState(); }

  decltype(auto) cost(internal::traits::pass_type<StateSpaceType> from,
                      internal::traits::pass_type<StateSpaceType> to) const
  {
    return CostType::cost(from, to);
  }
};

}  // namespace environment
}  // namespace anyangle