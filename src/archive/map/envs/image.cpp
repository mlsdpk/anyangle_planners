#include "anyangle_planners/map/envs/image.hpp"

namespace anyangle {
namespace map {

ImageEnvironment::ImageEnvironment(const std::string& path_to_image, const double resolution) : Environment()
{
  cv::Mat image;
  try
  {
    image = cv::imread(path_to_image, cv::IMREAD_GRAYSCALE);
  } catch (const std::exception& e)
  {
    std::cerr << "Error: failed to load image " << path_to_image << ": " << e.what() << '\n';
    exit(0);
  }

  // initialize gridmap
  grid_map::GridMapCvConverter::initializeFromImage(image, resolution, map_, grid_map::Position(0.0, 0.0));

  // create gridmap layer from image
  if (!grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, "layer", map_))
  {
    std::cerr << "Failed to create environment map from image " << path_to_image << std::endl;
    exit(0);
  }
}

bool ImageEnvironment::inCollision(const anyangle::graph::State2D& state) const noexcept;

anyangle::graph::State2DList ImageEnvironment::getValidNeighbors(const anyangle::graph::State2D& parent) const noexcept;

}  // namespace map
}  // namespace anyangle