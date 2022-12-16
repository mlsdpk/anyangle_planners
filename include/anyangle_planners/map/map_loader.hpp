#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <memory>

#include "anyangle_planners/map/environment.hpp"

namespace anyangle {
namespace map {

class MapLoader {
 public:
  MapLoader() = delete;
  explicit MapLoader(const std::string& path_to_map, EnvironmentType env_type);

  void toOccupancyGrid(nav_msgs::OccupancyGrid& ogm);

 private:
  EnvironmentPtr env_;
};

typedef std::shared_ptr<MapLoader> MapLoaderPtr;

}  // namespace map
}  // namespace anyangle