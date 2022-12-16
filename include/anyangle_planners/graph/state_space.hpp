#pragma once

#include "anyangle_planners/graph/vertex.hpp"

namespace anyangle {
namespace graph {

class State2D
{
  using State2DPtr = std::shared_ptr<State2D>;
  using State2DConstPtr = std::shared_ptr<const State2D>;
  using State2DList = std::vector<State2DPtr>;

public:
  State2D() {}
  State2D(const float& x, const float& y) : x(x), y(y) {}

  float x, y;
};

using State2DPtr = std::shared_ptr<State2D>;
using State2DList = std::vector<State2D>;

}  // namespace graph
}  // namespace anyangle