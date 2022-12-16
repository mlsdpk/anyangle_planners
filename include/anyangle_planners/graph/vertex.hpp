#pragma once

#include <memory>
#include <vector>

namespace anyangle {
namespace graph {

class Vertex
{
public:
  using VertexPtr = std::shared_ptr<Vertex>;
  using VertexConstPtr = std::shared_ptr<const Vertex>;
  using VertexList = std::vector<VertexPtr>;

  /**
   * @brief Deleted default constructor
   *
   */
  Vertex() = delete;

  /**
   * @brief Constructor
   *
   * @param _x x-coordinate
   * @param _y y-coordinate
   */
  explicit Vertex(const unsigned int _x, const unsigned int _y)
    : x_idx{_x}, y_idx{_y}, parent_vertex{nullptr}
  {
  }

  /**
   * @brief Default copy/move constructors and assignments
   *
   */
  Vertex(const Vertex&) = default;
  Vertex& operator=(const Vertex&) = default;
  Vertex(Vertex&&) = default;
  Vertex& operator=(Vertex&&) = default;

  /**
   * @brief Getter for the x-index of the vertex
   *
   * @return x-index
   */
  [[nodiscard]] const unsigned int x() const noexcept { return x_idx; }

  /**
   * @brief Getter for the x-index of the vertex
   *
   * @return x-index
   */
  [[nodiscard]] unsigned int& x() noexcept { return x_idx; }

  /**
   * @brief Getter for the y-index of the vertex
   *
   * @return y-index
   */
  [[nodiscard]] const unsigned int y() const noexcept { return y_idx; }

  /**
   * @brief Getter for the y-index of the vertex
   *
   * @return y-index
   */
  [[nodiscard]] unsigned int& y() noexcept { return y_idx; }

  /**
   * @brief Getter for the y-index of the vertex
   *
   * @return y-index
   */
  [[nodiscard]] const VertexConstPtr parent() const noexcept { return parent_vertex; }

  /**
   * @brief Getter for the y-index of the vertex
   *
   * @return y-index
   */
  [[nodiscard]] VertexPtr parent() noexcept { return parent_vertex; }

protected:
  /// @brief x-index of the vertex
  unsigned int x_idx;

  /// @brief y-index of the vertex
  unsigned int y_idx;

  /// @brief pointer to parent vertex of this vertex
  VertexPtr parent_vertex;
};

}  // namespace graph
}  // namespace anyangle