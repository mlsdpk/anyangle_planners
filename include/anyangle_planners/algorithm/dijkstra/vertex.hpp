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

#include "anyangle_planners/graph/point2d.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

template <typename T>
inline unsigned int toIndex(const T x, const T y, const T width)
{
  return (x * width) + y;
}

class Vertex : public graph::Point2D<unsigned int>
{
public:
  inline Vertex(const unsigned int _x, const unsigned int _y, const unsigned int _index)
    : graph::Point2D<unsigned int>(_x, _y), index{_index}
  {
  }

  unsigned int index;

  double f_cost{std::numeric_limits<double>::infinity()};
  double g_cost{std::numeric_limits<double>::infinity()};
  double h_cost{std::numeric_limits<double>::infinity()};

  bool visited{false};

  // - key to store in the priority queue
  //   <f-value, g-value>
  // - uses lexicographical ordering for minimum vertex selection
  std::pair<double, double> key;

  void updateKey()
  {
    f_cost = g_cost + h_cost;
    key = std::make_pair(f_cost, g_cost);
  }

  bool operator>(const Vertex& other) const
  {
    // std::pair support lexicographical comparison
    // which is convenient for us to implement the comparison
    // without writing additional code
    return key > other.key;
  }
};

}  // namespace algorithm::dijkstra
}  // namespace anyangle