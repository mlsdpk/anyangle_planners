#pragma once

#include "anyangle_planners/graph/vertex.hpp"
#include "anyangle_planners/utils/macros.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

ANYANGLE_OBJ_FORWARD(DijkstraVertex)

class DijkstraVertex : public Vertex
{
public:
  explicit DijkstraVertex(const unsigned int _x, const unsigned int _y, const unsigned int _index)
    : Vertex{_x, _y}, index{_index}
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
};

}  // namespace algorithm::dijkstra
}  // namespace anyangle