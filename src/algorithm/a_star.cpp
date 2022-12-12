#include "anyangle_planners/algorithm/a_star.hpp"

namespace anyangle {
namespace algorithm {

AStar::AStar(const std::string &name) : Planner(name) {}

void AStar::reset() {
  // clear the graph
  graph_.clear();

  // initialize ASTAR by clearing open list and add start node
  while (!open_list_.empty()) {
    open_list_.pop();
  }
}

AStar::VertexPtr AStar::addToGraph(const unsigned int &index) {
  auto vertex = std::make_shared<astar::Vertex>(index);
  return graph_.emplace(index, std::move(vertex)).first->second;
}

bool AStar::solve(const State2D &start, const State2D &goal) {
  // if no environment is given, return false
  if (!env_) {
    std::cerr << "No environment provided.\n";
    return false;
  }

  auto width = env_->getWidth();
  auto height = env_->getHeight();
  std::cout << "Received environment of size " << width << " x " << height << std::endl;

  // start and goal must be within bounds
  if (start.x < 0 || start.x >= width || goal.x < 0 || goal.x >= width || start.y < 0 ||
      start.y >= height || goal.x < 0 || goal.y >= height) {
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
  start_vertex_ = addToGraph(getNodeIndex(start.x, start.y, width));
  goal_vertex_ = addToGraph(getNodeIndex(goal.x, goal.y, width));

  // add start vertex into expansion queue (openlist)
  start_vertex_->g_cost = 0.0;
  start_vertex_->h_cost =
      costToGoHeuristics(toState2D(*start_vertex_, width), toState2D(*goal_vertex_, width));
  start_vertex_->updateKey();
  open_list_.push(start_vertex_);

  return true;
}

void AStar::getNodeExpansions(State2DList &nodes) {}

std::size_t AStar::getTotalMemory() { return 0u; }

}  // namespace algorithm
}  // namespace anyangle