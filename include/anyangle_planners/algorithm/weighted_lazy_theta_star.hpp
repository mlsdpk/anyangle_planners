#pragma once

#include "anyangle_planners/algorithm/theta_star.hpp"

namespace anyangle {
namespace algorithm {

class WeightedLazyThetaStar : public ThetaStar {
 public:
  explicit WeightedLazyThetaStar(const std::string& name);

  bool solve(const State2D& start, const State2D& goal) override;

  void getNodeExpansions(State2DList& nodes) override;

  void getStartNodeExpansions(State2DList& nodes);
  void getGoalNodeExpansions(State2DList& nodes);

  bool getSolutionPath(State2DList& path) const override;

  void reset() override;

 protected:
  bool lineOfSightWithIndex(const anyangle::State2D& from, const anyangle::State2D& to, int& x,
                            int& y) const;
  bool lineOfSightWithIndex(const astar::VertexConstPtr& from, const astar::VertexConstPtr& to,
                            int& x, int& y) const;

  //// openlist (frontier) for A* algorithm
  PQueueT start_open_list_;
  PQueueT goal_open_list_;

  //// closelist (expanded vertices)
  State2DList start_close_list_;
  State2DList goal_close_list_;

  std::unordered_map<astar::Vertex, double> forward_search_g_table_;
  std::unordered_map<astar::Vertex, double> backward_search_g_table_;
};

}  // namespace algorithm
}  // namespace anyangle