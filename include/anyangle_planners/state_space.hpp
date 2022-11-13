#pragma once

#include <vector>

namespace anyangle {

struct State2D {
  State2D() {}
  State2D(const float& x, const float& y) : x(x), y(y) {}

  float x, y;
};

using State2DList = std::vector<State2D>;

}  // namespace anyangle