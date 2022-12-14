#pragma once

#include "anyangle_planners/algorithm/a_star.hpp"

namespace anyangle {
namespace algorithm {

class ThetaStar : public AStar {
 public:
  explicit ThetaStar(const std::string &name);

  bool solve(const State2D &start, const State2D &goal) override;

  unsigned int getLineOfSightCheckCount() const { return line_of_sight_checks_; }

 protected:
  /**
   * @brief Bresenaham's raycasting algorithm to check line of sight between two vertices.
   *
   * @param from
   * @param to
   * @return true
   * @return false
   */
  bool lineOfSight(const astar::VertexConstPtr &from, const astar::VertexConstPtr &to) const;

  /**
   * @brief
   *
   * @param from
   * @param to
   * @return true
   * @return false
   */
  bool lineOfSight(const anyangle::State2D &from, const anyangle::State2D &to) const;

  unsigned int line_of_sight_checks_;
};

}  // namespace algorithm
}  // namespace anyangle