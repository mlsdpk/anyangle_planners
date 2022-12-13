#pragma once

#include "anyangle_planners/algorithm/theta_star.hpp"

namespace anyangle {
namespace algorithm {

class LazyThetaStar : public ThetaStar {
 public:
  explicit LazyThetaStar(const std::string &name);

  bool solve(const State2D &start, const State2D &goal) override;
};

}  // namespace algorithm
}  // namespace anyangle