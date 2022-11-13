#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <memory>

namespace anyangle {
namespace map {

enum class EnvironmentType { IMAGE = 0 };

class Environment {
 public:
  Environment() = default;

  bool initializeFromImage(const std::string& path_to_image,
                           const double resolution);

  bool inCollision(const float x, const float y) const;

 private:
  grid_map::GridMap map_;
};

typedef std::shared_ptr<Environment> EnvironmentPtr;

}  // namespace  map
}  // namespace anyangle