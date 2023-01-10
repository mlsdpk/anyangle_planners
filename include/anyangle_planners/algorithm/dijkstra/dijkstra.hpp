#pragma once

#include "anyangle_planners/algorithm/planner.hpp"

namespace anyangle {
namespace algorithm {

class Dijkstra : public Planner
{
  explicit Dijkstra(const std::string &name);
  ~Dijkstra() override;

  void reset() override;

  bool solve(const graph::State2D &start, const graph::State2D &goal) override;

  void getNodeExpansions([[maybe_unused]] graph::State2DList &nodes) const override;

  std::size_t getTotalMemory() const override;

  bool getSolutionPath(graph::State2DList &path) const override;

  double getPathCost() const override;
};

}  // namespace algorithm
}  // namespace anyangle
