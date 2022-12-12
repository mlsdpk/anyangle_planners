#include "anyangle_planners/algorithm/a_star.hpp"

namespace anyangle {
namespace algorithm {

AStar::AStar(const std::string &name) : Planner(name) {}

void AStar::reset() {
  // set 8-connectivity
  neighbors_grid_offsets_ = {-1,
                             +1,
                             -static_cast<int>(env_width_),
                             +static_cast<int>(env_width_),
                             -static_cast<int>(env_width_) - 1,
                             -static_cast<int>(env_width_) + 1,
                             +static_cast<int>(env_width_) - 1,
                             +static_cast<int>(env_width_) + 1};

  // clear the graph
  graph_.clear();

  // initialize ASTAR by clearing open list and add start node
  while (!open_list_.empty()) {
    open_list_.pop();
  }
}

astar::VertexPtr AStar::addToGraph(const unsigned int &index) {
  auto vertex = std::make_shared<astar::Vertex>(index);
  return graph_.emplace(index, std::move(vertex)).first->second;
}

bool AStar::solve(const State2D &start, const State2D &goal) {
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

  // add start vertex into expansion queue (openlist)
  start_vertex_->g_cost = 0.0;
  start_vertex_->h_cost = costToGoHeuristics(toState2D(*start_vertex_, env_width_),
                                             toState2D(*goal_vertex_, env_width_));
  start_vertex_->updateKey();
  open_list_.push(start_vertex_);

  bool solved = false;

  while (!open_list_.empty()) {
    astar::VertexPtr v_min = open_list_.top();
    v_min->visited = true;
    open_list_.pop();

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

      // calculate neighbor new g-value
      double neighbor_g_value = v_min->g_cost + distanceCost(*v_min, *neighbor);

      // if this new g-value is better than old
      if (neighbor_g_value < neighbor->g_cost) {
        // update this neighbor g-value, h-value and parent
        neighbor->parent_vertex = v_min;
        neighbor->g_cost = neighbor_g_value;
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

void AStar::getNodeExpansions(State2DList &nodes) {}

std::size_t AStar::getTotalMemory() { return 0u; }

bool AStar::getSolutionPath(State2DList &path) const {
  if (!goal_vertex_->parent_vertex) return false;

  astar::VertexConstPtr current_vertex = goal_vertex_;
  path.push_back(toState2D(*current_vertex, env_width_));
  while (current_vertex->parent_vertex != start_vertex_) {
    current_vertex = current_vertex->parent_vertex;
    path.push_back(toState2D(*current_vertex, env_width_));
  }
  path.push_back(toState2D(*start_vertex_, env_width_));

  return true;
}

void AStar::getNeighbors(astar::VertexPtr parent, astar::VertexList &neighbors) {
  auto parent_state2D = toState2D(*parent, env_width_);

  for (std::size_t i = 0; i != neighbors_grid_offsets_.size(); ++i) {
    auto index = parent->index + neighbors_grid_offsets_[i];

    // Check for wrap around conditions
    auto child = toState2D(index, env_width_);

    if (fabs(parent_state2D.x - child.x) > 1 || fabs(parent_state2D.y - child.y) > 1) {
      continue;
    }

    astar::VertexPtr neighbor;
    if (getValidNeighbor(index, neighbor)) {
      if (!env_->inCollision(toState2D(*neighbor, env_width_))) {
        neighbors.emplace_back(neighbor);
      }
    }
  }
}

bool AStar::getValidNeighbor(const unsigned int index, astar::VertexPtr &neighbor) {
  if (index < 0 || index >= (env_width_ * env_height_)) return false;
  if (env_->inCollision(toState2D(index, env_width_))) return false;

  neighbor = addToGraph(index);
  return true;
}

}  // namespace algorithm
}  // namespace anyangle