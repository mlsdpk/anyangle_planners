#pragma once

#include "anyangle_planners/graph/state_space.hpp"

namespace anyangle {
namespace utils {

/**
 * @brief Get the 1D index representation from x and y coordinates.
 *
 * @param x coordinate of node.
 * @param y coordinate of node.
 * @param width map width.
 * @return index
 */
[[nodiscard]] inline unsigned int getNodeIndex(const unsigned int x, const unsigned int y,
                                               const unsigned int width) noexcept
{
  return x + y * width;
}

/**
 * @brief Get the 1D index representation from the 2D node object.
 *
 * @param state 2D state object.
 * @param width map width.
 * @return index
 */
[[nodiscard]] inline unsigned int getNodeIndex(const graph::State2D &state,
                                               const unsigned int width) noexcept
{
  return getNodeIndex(static_cast<unsigned>(state.x), static_cast<unsigned>(state.y), width);
}

}  // namespace utils
}  // namespace anyangle