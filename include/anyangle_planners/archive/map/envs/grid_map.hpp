#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include "anyangle_planners/map/environment.hpp"

namespace anyangle {
namespace map {

/**
 * @brief
 *
 */
namespace grid_map_cost {
static constexpr float FREE = 0.0;
static constexpr float OCCUPIED = 1.0;
}  // namespace grid_map_cost

/**
 * @brief Two-dimensional grid map environment that search algorithms will be use against to do
 * exploration and planning. This environment depends on universal grid map library from ANYbotics
 * (https://github.com/ANYbotics/grid_map)
 *
 */
class GridMapEnvironment : public Environment
{
public:
  /**
   * @brief Different types of grid connectivity
   *
   */
  enum class Connectivity
  {
    FOUR,  // four-connected grid
    EIGHT  // eight-connected grid
  };

  GridMapEnvironment() = delete;

  /**
   * @brief Constructor
   *
   * @param connectivity
   * @param layer_name
   */
  explicit GridMapEnvironment(Connectivity connectivity, const std::string& layer_name);

  /**
   * @brief Given the state, this methods check whether the state is collision-free or not.
   *
   * @param state two-dimensional state
   * @return true if the state is occupied, false otherwise
   */
  [[nodiscard]] virtual bool inCollision(
      const anyangle::graph::State2D& state) const noexcept override;

  /**
   * @brief Method to get the valid neighbors (i.e., those that can be freely traverse from current
   * state) from the given 2D state.
   *
   * @param parent current state
   * @return list of freely traversable neighbors
   */
  [[nodiscard]] virtual anyangle::graph::State2DList getValidNeighbors(
      const anyangle::graph::State2D& parent) const noexcept override;

  /**
   * @brief Construct gridmap object from gridmap itself
   *
   * @param grid_map input gridmap
   */
  void create(const grid_map::GridMap& grid_map);

  /**
   * @brief Getter for gridmap object
   */
  grid_map::GridMap& getGridMap() { return map_; }

protected:
  /**
   * @brief Given the cell x and y indexes, this methods check whether the cell
   * is collision-free or not.
   *
   * @param x x-coordinate of the grid cell
   * @param y y-coordinate of the grid cell
   * @return true if the state is occupied, false otherwise
   */
  [[nodiscard]] virtual bool inCollision(const unsigned int x, const unsigned int y,
                                         const std::string& layer) const noexcept;

  /// @brief underlying data structure to store the gridmap information
  grid_map::GridMap map_;

  /// @brief width of the map in meters [m]
  double map_width_;

  /// @brief height of the map in meters [m]
  double map_height_;

  /// @brief type of grid-connectivity to be used
  Connectivity connectivity_;

  /// @brief pre-computed grid offsets based on type of grid-connectivity
  std::vector<int> neighbors_grid_offsets_;

  /// @brief layer name of the gridmap (only one layer is supported for now)
  std::string layer_name_{"layer"};
};

}  // namespace map
}  // namespace anyangle