#pragma once

#include <cmath>
#include <type_traits>

#include "anyangle_planners/graph/state_space.hpp"

namespace anyangle {
namespace utils {

/**
 * @brief Type of distance metric between two points
 *
 */
enum class DistanceType
{
  MANHATTAN,
  EUCLIDEAN,
  OCTILE,
  CHEBYSHEV
};

/**
 * @brief Standard L1 distance between two coordinates also known as Manhattan distance.
 *
 * @tparam T specialization for integral or floating point types
 * @param x1 from x-coordinate
 * @param y1 from y-coordinate
 * @param x2 to x-coordinate
 * @param y2 to y-coordinate
 * @return distance cost
 */
template <typename T>
[[nodiscard]] inline double manhattanDistance(const T x1, const T y1, const T x2,
                                              const T y2) noexcept
{
  static_assert((std::is_floating_point_v<T> || std::is_integral_v<T>),
                "Distance function only allows integral or floating point inputs.");

  return static_cast<double>(std::fabs(x1 - x2) + std::fabs(y1 - y2));
}

/**
 * @brief Standard L1 distance between two states also known as Manhattan distance.
 *
 * @param s1 from state
 * @param s2 to state
 * @return distance cost
 */
[[nodiscard]] inline double manhattanDistance(const graph::State2D &s1,
                                              const graph::State2D &s2) noexcept
{
  return manhattanDistance(s1.x, s1.y, s2.x, s2.y);
}

/**
 * @brief Standard L2 distance between two coordinates also known as Euclidean distance.
 *
 * @tparam T specialization for integral or floating point types
 * @param x1 from x-coordinate
 * @param y1 from y-coordinate
 * @param x2 to x-coordinate
 * @param y2 to y-coordinate
 * @return distance cost
 */
template <typename T>
[[nodiscard]] inline double euclideanDistance(const T x1, const T y1, const T x2,
                                              const T y2) noexcept
{
  static_assert((std::is_floating_point_v<T> || std::is_integral_v<T>),
                "Distance function only allows integral or floating point inputs.");

  return static_cast<double>(std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
}

/**
 * @brief Standard L2 distance between two states also known as Euclidean distance.
 *
 * @param s1 from state
 * @param s2 to state
 * @return distance cost
 */
[[nodiscard]] inline double euclideanDistance(const graph::State2D &s1,
                                              const graph::State2D &s2) noexcept
{
  return euclideanDistance(s1.x, s1.y, s2.x, s2.y);
}

/**
 * @brief Octile distance between two coordinates.
 *
 * @tparam T specialization for integral or floating point types
 * @param x1 from x-coordinate
 * @param y1 from y-coordinate
 * @param x2 to x-coordinate
 * @param y2 to y-coordinate
 * @return distance cost
 */
template <typename T>
[[nodiscard]] inline double octileDistance(const T x1, const T y1, const T x2, const T y2) noexcept
{
  static_assert((std::is_floating_point_v<T> || std::is_integral_v<T>),
                "Distance function only allows integral or floating point inputs.");

  const auto dx = std::fabs(x1 - x2);
  const auto dy = std::fabs(y1 - y2);

  if (dx > dy) return 1.0 * (dx - dy) + std::sqrt(2.0) * dy;
  return 1.0 * (dy - dx) + std::sqrt(2.0) * dx;
}

/**
 * @brief Octile distance between two states.
 *
 * @param s1 from state
 * @param s2 to state
 * @return distance cost
 */
[[nodiscard]] inline double octileDistance(const graph::State2D &s1,
                                           const graph::State2D &s2) noexcept
{
  return octileDistance(s1.x, s1.y, s2.x, s2.y);
}

}  // namespace utils
}  // namespace anyangle