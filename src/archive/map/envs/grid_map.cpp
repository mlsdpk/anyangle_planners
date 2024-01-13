#include "anyangle_planners/map/envs/grid_map.hpp"

namespace anyangle {
namespace map {

GridMapEnvironment::GridMapEnvironment(Connectivity connectivity, const std::string& layer_name)
  : connectivity_{connectivity}, layer_name_{layer_name}
{
}

void GridMapEnvironment::create(const grid_map::GridMap& grid_map)
{
  map_ = grid_map;

  // update map width and height
  map_width_ = map_.getLength()[0];
  map_height_ = map_.getLength()[1];

  // update grid offsets
  if (connectivity_ == Connectivity::FOUR)
  {
    // set 4-connectivity
    neighbors_grid_offsets_ = {-1,
                               +1,
                               -static_cast<int>(map_width_),
                               +static_cast<int>(map_width_),
                               -static_cast<int>(map_width_) - 1,
                               -static_cast<int>(map_width_) + 1,
                               +static_cast<int>(map_width_) - 1,
                               +static_cast<int>(map_width_) + 1};
  }
  else
  {
    // default to 8-connectivity
    neighbors_grid_offsets_ = {-1,
                               +1,
                               -static_cast<int>(map_width_),
                               +static_cast<int>(map_width_),
                               -static_cast<int>(map_width_) - 1,
                               -static_cast<int>(map_width_) + 1,
                               +static_cast<int>(map_width_) - 1,
                               +static_cast<int>(map_width_) + 1};
  }
}

bool GridMapEnvironment::inCollision(const anyangle::graph::State2D& state) const noexcept
{
  return inCollision(state.vertex().x, state.vertex().y, layer_name_);
}

anyangle::graph::State2DList GridMapEnvironment::getValidNeighbors(
    const anyangle::graph::State2D& parent) const noexcept
{
}

bool GridMapEnvironment::inCollision(const unsigned int x, const unsigned int y,
                                     const std::string& layer) const noexcept
{
  return map_.at(layer, grid_map::Index(x, y)) == grid_map_cost::OCCUPIED ? true : false;
}

}  // namespace map
}  // namespace anyangle