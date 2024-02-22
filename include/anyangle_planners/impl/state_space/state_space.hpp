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

#include <array>
#include <cstddef>

namespace anyangle {
namespace state_space {

/**
 * @brief Generic interface class to represent a state space in N-dimensions.
 *
 * Note that this class is based on the CRTP design pattern.
 *
 * @tparam Derived derived class
 * @tparam T datatype of state variables
 * @tparam Dimension dimension of state space
 */
template <typename Derived, typename T, std::size_t Dimension>
class StateSpaceBase
{
  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

public:
  using value_t = T;
  static constexpr size_t DIMENSION = Dimension;

  T& operator[](std::size_t index) { return state_variables_[index]; }

  const T& operator[](std::size_t index) const { return state_variables_[index]; }

  friend std::ostream& operator<<(std::ostream& os, const StateSpaceBase& state)
  {
    os << "State variables: ";
    for (std::size_t i = 0; i < Dimension; ++i)
    {
      os << state[i];
      if (i < Dimension - 1)
      {
        os << ", ";
      }
    }
    return os;
  }

protected:
  /// @brief Container to store all state variables
  std::array<T, Dimension> state_variables_;
};

}  // namespace state_space
}  // namespace anyangle