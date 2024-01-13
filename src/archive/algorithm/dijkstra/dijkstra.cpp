#pragma once

#include "anyangle_planners/algorithm/dijkstra.hpp"

namespace anyangle {
namespace algorithm::dijkstra {

Dijkstra::Dijkstra(const std::string &name) : Planner(name) {}

void Dijkstra::reset() {}

bool Dijkstra::solve(const graph::State2D &start, const graph::State2D &goal) {}

void Dijkstra::getNodeExpansions([[maybe_unused]] graph::State2DList &nodes) const {}

std::size_t Dijkstra::getTotalMemory() const {}

bool Dijkstra::getSolutionPath(graph::State2DList &path) const {}

double Dijkstra::getPathCost() const {}

}  // namespace algorithm::dijkstra
}  // namespace anyangle