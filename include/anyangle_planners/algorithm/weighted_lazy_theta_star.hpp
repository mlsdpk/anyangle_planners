#pragma once

#include "anyangle_planners/algorithm/theta_star.hpp"

namespace anyangle {
namespace algorithm {

class WeightedLazyThetaStar : public ThetaStar {
 public:
  explicit WeightedLazyThetaStar(const std::string& name);

  bool solve(const State2D& start, const State2D& goal) override;

 protected:
  bool lineOfSightWithIndex(const anyangle::State2D& from, const anyangle::State2D& to, int& x,
                            int& y) const;
  bool lineOfSightWithIndex(const astar::VertexConstPtr& from, const astar::VertexConstPtr& to,
                            int& x, int& y) const;
};

}  // namespace algorithm
}  // namespace anyangle