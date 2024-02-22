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

#include <type_traits>

namespace anyangle {

//////////////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS
//////////////////////////////////////////////////////////////////////////////////////

namespace graph {
template <typename Derived, typename T, std::size_t Dimension>
class StateSpaceBase;
}  // namespace graph

namespace environment {
template <typename Derived, typename StateSpaceType, typename CostType, typename>
class EnvironmentBase;
}  // namespace environment

//////////////////////////////////////////////////////////////////////////////////////

namespace internal::traits {

/// @brief Trait to check whether the class template is a derived class of
/// graph::StateSpaceBase
template <typename T>
using IsStateSpace = std::enable_if_t<
    std::is_base_of_v<graph::StateSpaceBase<T, typename T::value_t, T::DIMENSION>, T>>;

/// @brief Trait to check whether the class template is a derived class of
/// environment::EnvironmentBase
template <typename T>
using IsEnvironment = std::enable_if_t<std::is_base_of_v<
    environment::EnvironmentBase<T, typename T::state_space_t, typename T::cost_t, void>, T>>;

template <typename T>
using pass_type = std::conditional_t<std::is_fundamental_v<T>, T, const T&>;

}  // namespace internal::traits
}  // namespace anyangle