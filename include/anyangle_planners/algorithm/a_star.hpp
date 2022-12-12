#pragma once

#include <queue>

#include "anyangle_planners/algorithm/planner.hpp"

namespace anyangle {
namespace algorithm {
namespace astar {

class Vertex {
 public:
  explicit Vertex(const unsigned int index) : index{index} {}

  unsigned int index;

  double f_cost;
  double g_cost;
  double h_cost;

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

  void init() override;
  void clear() override;

  bool solve(const State2D &start, const State2D &goal) override;

  void getNodeExpansions(State2DList &nodes) override;

  std::size_t getTotalMemory() override;

  astar::VertexPtr addToGraph(const unsigned int &index);

  anyangle::State2D toState2D(const astar::Vertex &v, const unsigned int width) const {
    return anyangle::State2D(v.index % width, v.index / width);
  }

 private:
  using PQueueT =
      std::priority_queue<astar::VertexPtr, std::vector<astar::VertexPtr>, astar::VertexComparator>;

  //// openlist (frontier) for A* algorithm
  PQueueT open_list_;

  //// graph of processed vertices so far
  Graph graph_;

  //// start and goal vertices
  astar::VertexPtr start_vertex_;
  astar::VertexPtr goal_vertex_;
};

}  // namespace algorithm
}  // namespace anyangle