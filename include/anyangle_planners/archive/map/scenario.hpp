#pragma once

#include <string>
#include <vector>

namespace anyangle {
namespace map {
namespace moving_ai_lab {

/**
 * @brief
 *
 */
struct Scenario {
  /**
   * @brief
   *
   */
  struct Experiment {
    std::string map_name{""};
    unsigned int map_width{0};
    unsigned int map_height{0};
    unsigned int bucket{0};
    unsigned int start_x{0};
    unsigned int start_y{0};
    unsigned int goal_x{0};
    unsigned int goal_y{0};
    double optimal_length{0.0};
  };

  std::string name{""};
  std::string version{""};

  std::vector<Experiment> experiments;
};

}  // namespace moving_ai_lab
}  // namespace map
}  // namespace anyangle