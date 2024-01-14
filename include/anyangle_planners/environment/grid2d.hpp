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

#include "anyangle_planners/environment/environment.hpp"

namespace anyangle {
namespace environment {

template <typename StateSpaceType>
class Grid2D : public EnvironmentBase<Grid2D<StateSpaceType>, StateSpaceType>
{
public:
  inline Grid2D(const double width, const double height, const double resolution)
    : width_{width}, height_{height}, resolution_{resolution}
  {
    // TODO: create 2D grid cells
  }

  double getWidth() const { return width_; }
  double getHeight() const { return height_; }

  bool inCollision(const StateSpaceType& state) { return false; }

protected:
  double width_;
  double height_;
  double resolution_;
};

}  // namespace environment
}  // namespace anyangle