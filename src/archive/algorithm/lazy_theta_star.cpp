#include "anyangle_planners/algorithm/lazy_theta_star.hpp"

namespace anyangle {
namespace algorithm {

LazyThetaStar::LazyThetaStar(const std::string& name) : ThetaStar(name) {}

bool LazyThetaStar::solve(const State2D& start, const State2D& goal) {
  // if no environment is given, return false
  if (!env_) {
    std::cerr << "No environment provided.\n";
    return false;
  }

  env_width_ = env_->getWidth();
  env_height_ = env_->getHeight();
  std::cout << "Received environment of size " << env_width_ << " x " << env_height_ << std::endl;

  // start and goal must be within bounds
  if (start.x < 0 || start.x >= env_width_ || goal.x < 0 || goal.x >= env_width_ || start.y < 0 ||
      start.y >= env_height_ || goal.x < 0 || goal.y >= env_height_) {
    std::cerr << "Start and goal must be within bounds.\n";
    return false;
  }

  // start and goal must be collison free
  if (env_->inCollision(start) || env_->inCollision(goal)) {
    std::cerr << "Start and goal must be collision free.\n";
    return false;
  }

  // reset intenal stuffs
  reset();

  // add start and goal nodes into graph
  start_vertex_ = addToGraph(getNodeIndex(start.x, start.y, env_width_));
  goal_vertex_ = addToGraph(getNodeIndex(goal.x, goal.y, env_width_));

  std::cout << "Start idx: " << start_vertex_->index << std::endl;
  std::cout << "Goal idx: " << goal_vertex_->index << std::endl;

  line_of_sight_checks_ = 0u;

  // add start vertex into expansion queue (openlist)
  start_vertex_->g_cost = 0.0;
  start_vertex_->h_cost = costToGoHeuristics(toState2D(*start_vertex_, env_width_),
                                             toState2D(*goal_vertex_, env_width_));
  start_vertex_->updateKey();
  start_vertex_->parent_vertex = start_vertex_;

  open_list_.push(start_vertex_);

  bool solved = false;

  std::cout << "here\n";

  while (!open_list_.empty()) {
    astar::VertexPtr v_min = open_list_.top();
    v_min->visited = true;
    open_list_.pop();
    close_list_.push_back(toState2D(*v_min, env_width_));

    // if expanded vertex is goal, solved
    if (v_min == goal_vertex_) {
      solved = true;
      break;
    }

    // get all the neighbors that are collison-free and within map bounds
    astar::VertexList neighbors;
    getNeighbors(v_min, neighbors);

    // if no line of sight, reset the parent and connect to better neighbor
    if (!lineOfSight(v_min->parent_vertex, v_min)) {
      v_min->g_cost = std::numeric_limits<double>::infinity();
      for (auto itr = neighbors.begin(); itr != neighbors.end(); ++itr) {
        auto neighbor = *itr;

        double g_new_neighbor = neighbor->g_cost + distanceCost(*neighbor, *v_min);

        // if this neighbor is better, choose it as parent
        if (g_new_neighbor < v_min->g_cost) {
          // update this neighbor g-value, h-value and parent
          v_min->parent_vertex = neighbor;
          v_min->g_cost = g_new_neighbor;
          v_min->updateKey();
        }
      }
    }

    line_of_sight_checks_++;

    for (auto itr = neighbors.begin(); itr != neighbors.end(); ++itr) {
      auto neighbor = *itr;

      if (neighbor->visited) continue;

      const double g_new_neighbor =
          v_min->parent_vertex->g_cost + distanceCost(*v_min->parent_vertex, *neighbor);

      if (g_new_neighbor < neighbor->g_cost) {
        // update this neighbor g-value, h-value and parent
        neighbor->parent_vertex = v_min->parent_vertex;
        neighbor->g_cost = g_new_neighbor;
        neighbor->h_cost = costToGoHeuristics(toState2D(*neighbor, env_width_),
                                              toState2D(*goal_vertex_, env_width_));

        // add neighbor to priority queue
        neighbor->updateKey();
        open_list_.push(neighbor);
      }
    }
  }

  return solved;
}

}  // namespace algorithm
}  // namespace anyangle