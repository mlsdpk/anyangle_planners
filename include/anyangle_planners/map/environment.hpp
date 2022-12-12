#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <memory>
#include <opencv2/opencv.hpp>

#include "anyangle_planners/map/scenario.hpp"
#include "anyangle_planners/state_space.hpp"

namespace anyangle {
namespace map {

enum class EnvironmentType { IMAGE = 0, MovingAILabScenario = 1 };

class Environment {
 public:
  Environment() = default;

  bool initializeFromImage(const std::string& path_to_image, const double resolution);

  /**
   * @brief
   *
   * @param scenario
   * @return true
   * @return false
   */
  bool initializeFromMovingAILabScenario(const moving_ai_lab::Scenario& scenario);

  bool inCollision(const State2D& state) const;
  bool inCollision(const unsigned int x, const unsigned int y) const;

  unsigned int getWidth() const { return map_.getSize()(0); }

  unsigned int getHeight() const { return map_.getSize()(1); }

  cv::Mat convertToCVImageRGB();

  cv::Mat toImage();
  cv::Mat toImage(const anyangle::State2DList& path);

 private:
  grid_map::GridMap map_;
};

typedef std::shared_ptr<Environment> EnvironmentPtr;
typedef std::shared_ptr<const Environment> EnvironmentConstPtr;

}  // namespace  map
}  // namespace anyangle