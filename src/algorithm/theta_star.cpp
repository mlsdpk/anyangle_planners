#include "anyangle_planners/algorithm/theta_star.hpp"

namespace anyangle {
namespace algorithm {

ThetaStar::ThetaStar(const std::string& name) : AStar(name) {}

bool ThetaStar::solve(const State2D& start, const State2D& goal) {
  // if no environment is given, return false
  if (!env_) {
    std::cerr << "No environment provided.\n";
    return false;
  }

  env_width_ = env_->getWidth();
  env_height_ = env_->getHeight();
  // std::cout << "Received environment of size " << env_width_ << " x " << env_height_ <<
  // std::endl;

  // start and goal must be within bounds
  if (start.x < 0 || start.x >= env_width_ || goal.x < 0 || goal.x >= env_width_ || start.y < 0 ||
      start.y >= env_height_ || goal.x < 0 || goal.y >= env_height_) {
    // std::cerr << "Start and goal must be within bounds.\n";
    return false;
  }

  // start and goal must be collison free
  if (env_->inCollision(start) || env_->inCollision(goal)) {
    // std::cerr << "Start and goal must be collision free.\n";
    return false;
  }

  // reset intenal stuffs
  reset();

  // add start and goal nodes into graph
  start_vertex_ = addToGraph(getNodeIndex(start.x, start.y, env_width_));
  goal_vertex_ = addToGraph(getNodeIndex(goal.x, goal.y, env_width_));

  // std::cout << "Start idx: " << start_vertex_->index << std::endl;
  // std::cout << "Goal idx: " << goal_vertex_->index << std::endl;

  // add start vertex into expansion queue (openlist)
  start_vertex_->g_cost = 0.0;
  start_vertex_->h_cost = costToGoHeuristics(toState2D(*start_vertex_, env_width_),
                                             toState2D(*goal_vertex_, env_width_));
  start_vertex_->updateKey();
  start_vertex_->parent_vertex = start_vertex_;

  open_list_.push(start_vertex_);

  bool solved = false;

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

    for (auto itr = neighbors.begin(); itr != neighbors.end(); ++itr) {
      auto neighbor = *itr;

      if (neighbor->visited) continue;

      if (lineOfSight(v_min->parent_vertex, neighbor)) {
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
      } else {
        // calculate neighbor new g-value
        double g_new_neighbor = v_min->g_cost + distanceCost(*v_min, *neighbor);

        // if this new g-value is better than old
        if (g_new_neighbor < neighbor->g_cost) {
          // update this neighbor g-value, h-value and parent
          neighbor->parent_vertex = v_min;
          neighbor->g_cost = g_new_neighbor;
          neighbor->h_cost = costToGoHeuristics(toState2D(*neighbor, env_width_),
                                                toState2D(*goal_vertex_, env_width_));

          // add neighbor to priority queue
          neighbor->updateKey();
          open_list_.push(neighbor);
        }
      }
    }
  }

  return solved;
}

bool ThetaStar::lineOfSight(const astar::VertexConstPtr& from,
                            const astar::VertexConstPtr& to) const {
  return lineOfSight(toState2D(*from, env_width_), toState2D(*to, env_width_));
}

bool ThetaStar::lineOfSight(const anyangle::State2D& from, const anyangle::State2D& to) const {
  const int x0 = from.x;
  const int y0 = from.y;
  const int x1 = to.x;
  const int y1 = to.y;

  int cx, cy;
  int dy = abs(y1 - y0), dx = abs(x1 - x0), f = 0;
  int sx, sy;
  sx = x1 > x0 ? 1 : -1;
  sy = y1 > y0 ? 1 : -1;

  int u_x = (sx - 1) / 2;
  int u_y = (sy - 1) / 2;
  cx = x0;
  cy = y0;

  if (dx >= dy) {
    while (cx != x1) {
      f += dy;
      if (f >= dx) {
        if (env_->inCollision(cx + u_x, cy + u_y)) {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && env_->inCollision(cx + u_x, cy + u_y)) {
        return false;
      }
      if (dy == 0 && env_->inCollision(cx + u_x, cy) && env_->inCollision(cx + u_x, cy - 1)) {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1) {
      f = f + dx;
      if (f >= dy) {
        if (env_->inCollision(cx + u_x, cy + u_y)) {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && env_->inCollision(cx + u_x, cy + u_y)) {
        return false;
      }
      if (dx == 0 && env_->inCollision(cx, cy + u_y) && env_->inCollision(cx - 1, cy + u_y)) {
        return false;
      }
      cy += sy;
    }
  }
  return true;
}

}  // namespace algorithm
}  // namespace anyangle