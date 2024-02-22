// MIT License

// Copyright (c) 2024 Phone Thiha Kyaw

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <vector>

#include "anyangle_planners/impl/environment/environment.hpp"
#include "anyangle_planners/impl/state_space/state_space.hpp"

namespace anyangle {
namespace environment::graph {

//////////////////////////////////////////////////////////////////////////////////////////////

using vertex_id_t = std::uint32_t;

template <typename Derived = void>
struct VertexPropertiesBase
{
  vertex_id_t parent{};
};

template <typename Derived = void>
struct EdgePropertiesBase
{
  double weight{0.0};
};

class VertexStateSpace : public state_space::StateSpaceBase<VertexStateSpace, vertex_id_t, 1u>
{
public:
  VertexStateSpace() = default;
  explicit VertexStateSpace(const vertex_id_t q) { this->state_variables_[0] = q; }
};

using state_space_t = VertexStateSpace;

//////////////////////////////////////////////////////////////////////////////////////////////

template <typename CostFunctionType, typename StateSpaceType = state_space_t,
          typename VertexT = VertexPropertiesBase<void>, typename EdgeT = EdgePropertiesBase<void>>
class Graph : public EnvironmentBase<Graph<CostFunctionType, StateSpaceType, VertexT, EdgeT>,
                                     state_space_t, CostFunctionType, void>
{
public:
  using state_space_t = StateSpaceType;

  using Edge = std::pair<vertex_id_t, EdgeT>;
  Graph() = default;
  explicit Graph(const std::size_t graph_order) { this->vertices_.reserve(graph_order); }

  bool inCollision(const state_space_t& state) { return false; }

  void addVertex(const vertex_id_t q, const VertexT& vertex_properties)
  {
    if (q >= vertices_.size())
    {
      vertices_.resize(q + 1);
    }

    vertices_[q] = vertex_properties;
  }

  void addEdge(const vertex_id_t from, const vertex_id_t to, const EdgeT& edge_properties)
  {
    if (from >= adjacencies_.size())
    {
      adjacencies_.resize(from + 1);
    }

    this->adjacencies_[from].emplace_back(to, edge_properties);
  }

  void clear()
  {
    vertices_.clear();
    adjacencies_.clear();
  }

  size_t graphOrder() const noexcept { return vertices_.size(); }

  VertexT& vertex(const vertex_id_t q) { return vertices_[q]; }
  const VertexT& vertex(const vertex_id_t q) const { return vertices_[q]; }

  std::size_t vertexCount() const { return vertices_.size(); }

  template <typename NeighborsVisitorT>
  void forEachNeighbors(const vertex_id_t q, NeighborsVisitorT&& visitor)
  {
    const auto& edges = adjacencies_[q];
    std::for_each(edges.begin(), edges.end(),
                  [visitor](const auto& id_and_edge_properties) mutable {
                    std::apply(visitor, id_and_edge_properties);
                  });
  }

  void setStartAndGoalState(const state_space_t& start, const state_space_t& goal)
  {
    start_vertex_id_ = start[0];
    goal_vertex_id_ = goal[0];
  }

  decltype(auto) getStartAndGoalState() const
  {
    return std::make_pair(start_vertex_id_, goal_vertex_id_);
  }

protected:
  std::vector<VertexT> vertices_{};
  std::vector<std::vector<Edge>> adjacencies_{};
  vertex_id_t start_vertex_id_{};
  vertex_id_t goal_vertex_id_{};
};

}  // namespace environment::graph
}  // namespace anyangle