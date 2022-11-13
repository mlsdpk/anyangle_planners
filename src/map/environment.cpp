#include "anyangle_planners/map/environment.hpp"

namespace anyangle {
namespace map {

bool Environment::initializeFromImage(const std::string& path_to_image,
                                      const double resolution) {
  cv::Mat image;
  try {
    image = cv::imread(path_to_image, cv::IMREAD_GRAYSCALE);
  } catch (const std::exception& e) {
    std::cerr << "Error: failed to load image " << path_to_image << ": "
              << e.what() << '\n';
  }

  // initialize gridmap
  grid_map::GridMapCvConverter::initializeFromImage(
      image, resolution, map_, grid_map::Position(0.0, 0.0));

  // create gridmap layer from image
  if (!grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
          image, layer, map_)) {
    std::cerr << "Failed to create environment map from image " << path_to_image
              << std::endl;
  }
}

bool Environment::inCollision(const float x, const float y) const {
  return false;
}

}  // namespace map
}  // namespace anyangle