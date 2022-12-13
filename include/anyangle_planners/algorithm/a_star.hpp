#pragma once

#include <limits>
#include <queue>

#include "anyangle_planners/algorithm/planner.hpp"

namespace anyangle {
namespace algorithm {
namespace astar {

class Vertex {
 public:
  explicit Vertex(const unsigned int index) : index{index} {}

  ////
  unsigned int index;

  ////
  double f_cost{std::numeric_limits<double>::infinity()};
  double g_cost{std::numeric_limits<double>::infinity()};
  double h_cost{std::numeric_limits<double>::infinity()};

  bool visited{false};

  //// parent vertex to this vertex
  std::shared_ptr<Vertex> parent_vertex;

  // - key to store in the priority queue
  //   <f-value, g-value>
  // - uses lexicographical ordering for minimum vertex selection
  std::pair<double, double> key;

  /**
   * @brief
   *
   */
  void updateKey() {
    f_cost = g_cost + h_cost;
    key = std::make_pair(f_cost, g_cost);
  }
};

typedef std::shared_ptr<Vertex> VertexPtr;
typedef std::vector<VertexPtr> VertexList;
typedef std::shared_ptr<const Vertex> VertexConstPtr;

// custom function for returning minimum distance vertex
// to be used in priority queue
struct VertexComparator {
  // operator overloading
  bool operator()(const VertexConstPtr v1, const VertexConstPtr v2) const {
    // std::pair support lexicographical comparison
    // which is convenient for us to implement the comparison
    // without writing additional code
    return v1->key > v2->key;
  }
};

}  // namespace astar

class AStar : public Planner {
 public:
  using Graph = std::unordered_map<unsigned int, astar::VertexPtr>;

  /**
   * @brief Construct a new AStar object
   *
   * @param name
   */
  explicit AStar(const std::string &name);

  void reset() override;

  bool solve(const State2D &start, const State2D &goal) override;

  void getNodeExpansions(State2DList &nodes) override;

  std::size_t getTotalMemory() override;

  bool getSolutionPath(State2DList &path) const override;

  double getPathCost() const override { return goal_vertex_->g_cost; }

  astar::VertexPtr addToGraph(const unsigned int &index);

  anyangle::State2D toState2D(const astar::Vertex &v, const unsigned int width) const {
    return anyangle::State2D(v.index % width, v.index / width);
  }

  anyangle::State2D toState2D(const unsigned int index, const unsigned int width) const {
    return anyangle::State2D(index % width, index / width);
  }

  inline double distanceCost(const astar::Vertex &v1, const astar::Vertex &v2) const {
    // return Planner::octileDistance(toState2D(v1, env_width_), toState2D(v2, env_width_));
    return Planner::distanceCost(toState2D(v1, env_width_), toState2D(v2, env_width_));
  }

  void getNeighbors(astar::VertexPtr parent, astar::VertexList &neighbors);

  /**
   * @brief Get the Valid Neighbor object
   *
   * @param index
   * @param neighbor
   * @return true
   * @return false
   */
  bool getValidNeighbor(const unsigned int index, astar::VertexPtr &neighbor);

 protected:
  using PQueueT =
      std::priority_queue<astar::VertexPtr, std::vector<astar::VertexPtr>, astar::VertexComparator>;

  //// openlist (frontier) for A* algorithm
  PQueueT open_list_;

  //// closelist (expanded vertices)
  State2DList close_list_;

  //// graph of processed vertices so far
  Graph graph_;

  //// offsets for 8-connectivity
  std::vector<int> neighbors_grid_offsets_;

  //// start and goal vertices
  astar::VertexPtr start_vertex_;
  astar::VertexPtr goal_vertex_;

  //// environment width and height
  unsigned int env_width_;
  unsigned int env_height_;
};

}  // namespace algorithm
}  // namespace anyangle