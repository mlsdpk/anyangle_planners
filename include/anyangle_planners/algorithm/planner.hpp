#pragma once

#include <exception>
#include <memory>
#include <string>

#include "anyangle_planners/map/environment.hpp"
#include "anyangle_planners/state_space.hpp"

namespace anyangle {
namespace algorithm {

class Planner {
 public:
  // non-copyable
  Planner(const Planner &) = delete;
  Planner &operator=(const Planner &) = delete;

  /**
   * @brief Constructor
   *
   * @param name
   */
  explicit Planner(const std::string &name);

  /**
   * @brief Default destructor
   *
   */
  virtual ~Planner() = default;

  /**
   * @brief
   *
   */
  virtual void init() = 0;

  /**
   * @brief
   *
   */
  virtual void clear() = 0;

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  virtual bool solve([[maybe_unused]] const State2D &start,
                     [[maybe_unused]] const State2D &goal) = 0;

  /**
   * @brief Get the Node Expansions object
   *
   * @param nodes
   */
  virtual void getNodeExpansions([[maybe_unused]] State2DList &nodes) = 0;

  /**
   * @brief Get the Total Memory object
   *
   * @return std::size_t
   */
  virtual std::size_t getTotalMemory() = 0;

  void setEnvironment(map::EnvironmentConstPtr env) { env_ = env; }

  /**
   * @brief Get the Solution Path object
   *
   * @return State2DList
   */
  virtual State2DList getSolutionPath() const { return path_; }

  /**
   * @brief Get the Path Cost object
   *
   * @return double
   */
  virtual double getPathCost() const { return 0.0; }

  inline unsigned int getNodeIndex(const unsigned int &x, const unsigned int &y,
                                   const unsigned int &width) {
    return x + y * width;
  }

  double costToGoHeuristics(const State2D &s1, const State2D &s2) const {
    return std::sqrt((s1.x - s2.x) * (s1.x - s2.x) + (s1.y - s2.y) * (s1.y - s2.y));
  }

 protected:
  //// The name of the planner
  std::string name_;

  //// Solution path
  State2DList path_;

  //// Shared ptr to environment
  map::EnvironmentConstPtr env_;
};

typedef std::shared_ptr<Planner> PlannerPtr;

}  // namespace algorithm
}  // namespace anyangle