#pragma once

#include <exception>
#include <memory>
#include <string>

#include "anyangle_planners/graph/state_space.hpp"
#include "anyangle_planners/map/environment.hpp"
#include "anyangle_planners/utils/conversions.hpp"
#include "anyangle_planners/utils/distance.hpp"

namespace anyangle {
namespace algorithm {

class Planner
{
public:
  // non-copyable
  Planner(const Planner &) = delete;
  Planner &operator=(const Planner &) = delete;

  /**
   * @brief Constructor
   *
   * @param name Name of the planner
   */
  explicit Planner(const std::string &name);

  /**
   * @brief Default destructor
   *
   */
  virtual ~Planner() = default;

  /**
   * @brief Function for resetting internal data structures.
   *
   */
  virtual void reset() = 0;

  /**
   * @brief Main planning function to be implemented by the derived classes.
   *
   * @return true if planning is solved otherwise false
   */
  virtual bool solve([[maybe_unused]] const graph::State2D &start,
                     [[maybe_unused]] const graph::State2D &goal) = 0;

  /**
   * @brief Get the nodes expanded by the planning algorithm.
   *
   * @param nodes expanded nodes.
   */
  virtual void getNodeExpansions([[maybe_unused]] graph::State2DList &nodes) const = 0;

  /**
   * @brief Get the total memory usage of the planner.
   *
   * @return std::size_t
   */
  [[nodiscard]] virtual std::size_t getTotalMemory() const = 0;

  /**
   * @brief Set the planning environment.
   *
   * @param env User-defined environment map.
   */
  void setEnvironment(map::EnvironmentConstPtr env) { env_ = env; }

  /**
   * @brief Get the solution path after planning is succeeded.
   *
   * @return True if solution path exists False otherwise.
   */
  [[nodiscard]] virtual bool getSolutionPath([[maybe_unused]] graph::State2DList &path) const = 0;

  /**
   * @brief Get the cost of the solution path.
   *
   * @return solution cost.
   */
  [[nodiscard]] virtual double getPathCost() const = 0;

protected:
  //// The name of the planner
  std::string name_;

  //// Solution path
  graph::State2DList path_;

  //// Shared ptr to environment
  map::EnvironmentConstPtr env_;
};

typedef std::shared_ptr<Planner> PlannerPtr;

}  // namespace algorithm
}  // namespace anyangle