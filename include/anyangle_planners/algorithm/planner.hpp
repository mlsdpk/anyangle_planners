#pragma once

#include <memory>
#include <string>

#include "anyangle_planners/map/environment.hpp"
#include "anyangle_planners/state_space.hpp"

namespace anyangle {
namespace algorithm {

using CollisionCheckerFn = std::function<bool(const State2D *)>;

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

  /**
   * @brief Set the Collision Checking function. The function must return True
   * if the state is in collision.
   *
   * @param collision_checker User specific collision checking function
   */
  void setCollisionChecker(const CollisionCheckerFn &collision_checker) {
    if (!collision_checker)
      throw std::exception(
          "Invalid function definition for collision checking");

    collision_checker_ = std::move(collision_checker);
  }

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

 protected:
  /**
   * @brief
   *
   * @param state
   * @return true
   * @return false
   */
  bool isCollision(const State2D &state) const {
    return collision_checker_(state);
  }

  //// The name of the planner
  std::string name_;

  //// Solution path
  State2DList path_;

  //// Collision checking function
  CollisionCheckerFn collision_checker_;
};

}  // namespace algorithm
}  // namespace anyangle