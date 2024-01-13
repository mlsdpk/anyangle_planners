#pragma once

#include <limits>
#include <queue>
#include <unordered_map>

#include "anyangle_planners/algorithm/dijkstra/include/vertex.hpp"
#include "anyangle_planners/algorithm/planner.hpp"
#include "anyangle_planners/graph/state_space.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

class Dijkstra : public Planner
{
public:
  /**
   * @brief Constructor
   *
   * @param name Name of the planner
   */
  explicit Dijkstra(const std::string &name);

  /**
   * @brief Function for resetting internal data structures.
   *
   */
  void reset() override;

  /**
   * @brief Main planning function to be implemented by the derived classes.
   *
   * @return true if planning is solved otherwise false
   */
  bool solve(const graph::State2D &start, const graph::State2D &goal) override;

  /**
   * @brief Get the nodes expanded by the planning algorithm.
   *
   * @param nodes expanded nodes.
   */
  void getNodeExpansions([[maybe_unused]] graph::State2DList &nodes) const override;

  /**
   * @brief Get the total memory usage of the planner.
   *
   * @return std::size_t
   */
  std::size_t getTotalMemory() const override;

  /**
   * @brief Get the solution path after planning is succeeded.
   *
   * @return True if solution path exists False otherwise.
   */
  bool getSolutionPath(graph::State2DList &path) const override;

  /**
   * @brief Get the cost of the solution path.
   *
   * @return solution cost.
   */
  double getPathCost() const override;

protected:
  // custom function for returning minimum distance vertex
  // to be used in priority queue
  struct VertexComparator
  {
    // operator overloading
    bool operator()(const DijkstraVertexConstPtr v1, const DijkstraVertexConstPtr v2) const
    {
      // std::pair support lexicographical comparison
      // which is convenient for us to implement the comparison
      // without writing additional code
      return v1->key > v2->key;
    }
  };

  using PQueueT =
      std::priority_queue<DijkstraVertexPtr, std::vector<DijkstraVertexPtr>, VertexComparator>;
  using Graph = std::unordered_map<unsigned int, DijkstraVertexPtr>;

  //// openlist (frontier) for dijkstra algorithm
  PQueueT open_list_;

  //// closelist (expanded vertices)
  graph::State2DList close_list_;

  //// graph of processed vertices so far
  Graph graph_;
};

}  // namespace algorithm::dijkstra
}  // namespace anyangle