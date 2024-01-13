#pragma once

#include "anyangle_planners/graph/vertex.hpp"
#include "anyangle_planners/utils/macros.hpp"

namespace anyangle {
namespace graph {

ANYANGLE_OBJ_FORWARD(State2D)

class State2D
{
public:
  State2D() {}
  State2D(const float _x, const float _y) : x(_x), y(_y) {}

  float x, y;
};

typedef std::vector<State2D> State2DList;

}  // namespace graph
}  // namespace anyangle