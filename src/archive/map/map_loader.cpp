#include "anyangle_planners/map/map_loader.hpp"

namespace anyangle {
namespace map {

MapLoader::MapLoader(const std::string& path_to_map, EnvironmentType env_type) {
  env_ = std::make_shared<Environment>();
  switch (env_type) {
    case EnvironmentType::IMAGE:
      env_->initializeFromImage(path_to_map, 0.05);
      break;

    default:
      break;
  }
}

}  // namespace map
}  // namespace anyangle