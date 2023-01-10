#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include "anyangle_planners/map/environment.hpp"

namespace anyangle {
namespace map {

namespace grid_map_cost {
static constexpr float FREE = 0.0;
static constexpr float OCCUPIED = 1.0;
}  // namespace grid_map_cost

/**
 * @brief
 *
 */
class GridMapEnvironment : public Environment
{
public:
  /**
   * @brief
   *
   */
  GridMapEnvironment() = delete;

  /**
   * @brief
   *
   */
  enum class Connectivity
  {
    FOUR,
    EIGHT
  };

  /**
   * @brief Construct a new Grid Map Environment object
   *
   * @param connectivity
   * @param layer_name
   */
  explicit GridMapEnvironment(Connectivity connectivity, const std::string& layer_name);

  /**
   * @brief
   *
   * @param state
   * @return true
   * @return false
   */
  [[nodiscard]] virtual bool inCollision(
      const anyangle::graph::State2D& state) const noexcept override;

  /**
   * @brief Get the Valid Neighbors object
   *
   * @param parent
   * @return anyangle::graph::State2DList
   */
  [[nodiscard]] virtual anyangle::graph::State2DList getValidNeighbors(
      const anyangle::graph::State2D& parent) const noexcept override;

  /**
   * @brief
   *
   * @param grid_map
   */
  void updateMap(const grid_map::GridMap& grid_map);

  grid_map::GridMap& getGridMap() { return map_; }

protected:
  /**
   * @brief
   *
   * @param x
   * @param y
   * @return true
   * @return false
   */
  [[nodiscard]] virtual bool inCollision(const unsigned int x, const unsigned int y,
                                         const std::string& layer) const noexcept;

  /// @brief
  grid_map::GridMap map_;

  /// @brief
  double map_width_;
  double map_height_;

  /// @brief
  Connectivity connectivity_;

  /// @brief
  std::vector<int> neighbors_grid_offsets_;

  /// @brief layer name of the gridmap (only one layer is supported for now)
  std::string layer_name_{"layer"};
};

}  // namespace map
}  // namespace anyangle