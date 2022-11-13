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

  double g_cost;
  double h_cost;

  // - key to store in the priority queue
  //   <f-value, g-value>
  // - uses lexicographical ordering for minimum vertex selection
  std::pair<double, double> key;
};

// custom function for returning minimum distance node
// to be used in priority queue
struct MinimumDistanceASTAR {
  // operator overloading
  bool operator()(const Vertex *v1, const Vertex *v2) const {
    // std::pair support lexicographical comparison
    // which is convenient for us to implement the comparison
    // without writing additional code
    return v1->key > v2->key;
  }
};

}  // namespace astar

class AStar : public Planner {
 public:
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

 private:
  using PQueueT =
      std::priority_queue<astar::Vertex *, std::vector<astar::Vertex *>,
                          astar::MinimumDistanceASTAR>;

  PQueueT open_list_;
};

}  // namespace algorithm
}  // namespace anyangle