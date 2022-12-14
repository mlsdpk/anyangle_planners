#include "anyangle_planners/map/environment.hpp"

namespace anyangle {
namespace map {

bool Environment::initializeFromImage(const std::string& path_to_image, const double resolution) {
  cv::Mat image;
  try {
    image = cv::imread(path_to_image, cv::IMREAD_GRAYSCALE);
  } catch (const std::exception& e) {
    std::cerr << "Error: failed to load image " << path_to_image << ": " << e.what() << '\n';
  }

  // initialize gridmap
  grid_map::GridMapCvConverter::initializeFromImage(image, resolution, map_,
                                                    grid_map::Position(0.0, 0.0));

  // create gridmap layer from image
  if (!grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, "layer", map_)) {
    std::cerr << "Failed to create environment map from image " << path_to_image << std::endl;
  }
}

bool Environment::initializeFromMovingAILabScenario(const moving_ai_lab::Scenario& scenario) {
  // check the scenario version is supported or not
  if (scenario.version != "1") {
    std::cerr << "Only version 1 is currently supported for Moving AI Lab scenario.\n";
    return false;
  }

  // must contain at least one experiment
  if (scenario.experiments.empty()) {
    std::cerr << "Scenario must contain at least one experiment.\n";
    return false;
  }

  // here we assume that all the experiments inside scenario share the same map name
  // so we only use the first experiment map name
  std::string map_name = scenario.experiments.front().map_name;

  std::ifstream map_file(map_name);
  if (map_file.fail() && scenario.name.find('/') != std::string::npos) {
    map_name = scenario.name.substr(0, scenario.name.find_last_of('/')) + "/" + map_name;
  }

  std::cout << "Loading Moving AI Lab scenario map from " << map_name << std::endl;
  map_file = std::ifstream(map_name);

  if (map_file.fail()) {
    std::cerr << "Failed to load map file from " << map_name << std::endl;
  }

  std::string temp;
  std::getline(map_file, temp);

  std::size_t height;
  std::size_t width;

  map_file >> temp >> height;
  map_file >> temp >> width;
  map_file >> temp;
  std::getline(map_file, temp);

  // read the map and store inside temporary vector
  std::vector<bool> temp_map;
  temp_map.reserve(width * height);

  for (std::string line; std::getline(map_file, line);) {
    for (std::string::iterator it = line.begin(); it != line.end(); ++it) {
      if ((*it) == '.') {
        temp_map.push_back(true);
      } else if ((*it) == '@') {
        temp_map.push_back(false);
      }
    }
  }

  // initialize gridmap
  grid_map::Length length(height, width);
  map_.setGeometry(length, 1.0, grid_map::Position(0.0, 0.0));

  map_.add("layer");
  grid_map::Matrix& data = map_["layer"];

  std::size_t i = 0u;
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    const grid_map::Index imageIndex = iterator.getUnwrappedIndex();
    data(gridMapIndex(1), gridMapIndex(0)) = temp_map[i] ? 1.0 : 0.0;
    i++;
  }

  // cv::Mat img;
  // grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map_, "layer", CV_8U, img);
  // cv::transpose(img, img);
  // cv::imwrite("test.jpg", img);

  return true;
}

bool Environment::inCollision(const State2D& state) const {
  return inCollision(static_cast<unsigned>(state.x), static_cast<unsigned>(state.y));
}

bool Environment::inCollision(const unsigned int x, const unsigned int y) const {
  return map_.at("layer", grid_map::Index(x, y)) == 0.0 ? true : false;
}

cv::Mat Environment::convertToCVImageRGB() {
  cv::Mat img, img_rgb;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map_, "layer", CV_8U, img);

  // cv::transpose(img, img);

  cv::cvtColor(img, img_rgb, CV_GRAY2BGR);
  return img_rgb;
}

cv::Mat Environment::toImage() { return convertToCVImageRGB(); }

// cv::Mat Environment::toImage(const anyangle::State2DList& path) {
//   auto img = toImage();

//   for (std::size_t i = 0; i < path.size() - 1; ++i) {
//     cv::line(img, cv::Point(static_cast<int>(path[i].x), static_cast<int>(path[i].y)),
//              cv::Point(static_cast<int>(path[i + 1].x), static_cast<int>(path[i + 1].y)),
//              cv::Scalar(0, 255, 0), 1, cv::LINE_8);
//   }

//   // for (const auto& coor : path) {
//   //   img.at<cv::Vec<unsigned char, 3>>(static_cast<int>(coor.x), static_cast<int>(coor.y))[0] =
//   0;
//   //   img.at<cv::Vec<unsigned char, 3>>(static_cast<int>(coor.x), static_cast<int>(coor.y))[1] =
//   //   255; img.at<cv::Vec<unsigned char, 3>>(static_cast<int>(coor.x),
//   static_cast<int>(coor.y))[2]
//   //   = 0;

//   //   cv::line(img, cv::Point(static_cast<int>(coor.x), static_cast<int>(coor.y)), p4,
//   //            Scalar(255, 0, 0), thickness, LINE_8);
//   // }
//   return img;
// }

cv::Mat Environment::toImage(const anyangle::State2DList& expansions) {
  auto img = toImage();
  for (const auto& v : expansions) {
    img.at<cv::Vec<unsigned char, 3>>(static_cast<int>(v.x), static_cast<int>(v.y))[0] = 0;
    img.at<cv::Vec<unsigned char, 3>>(static_cast<int>(v.x), static_cast<int>(v.y))[1] = 0;
    img.at<cv::Vec<unsigned char, 3>>(static_cast<int>(v.x), static_cast<int>(v.y))[2] = 255;
  }
  return img;
}

cv::Mat Environment::toImage(const anyangle::State2DList& path,
                             const anyangle::State2DList& expansions) {
  // create expansions image first
  auto img = toImage(expansions);

  // draw path on it
  drawPath(path, img);

  return img;
}

void Environment::drawPath(const anyangle::State2DList& path, cv::Mat& img) {
  for (std::size_t i = 0; i < path.size() - 1; ++i) {
    cv::line(img, cv::Point(static_cast<int>(path[i].y), static_cast<int>(path[i].x)),
             cv::Point(static_cast<int>(path[i + 1].y), static_cast<int>(path[i + 1].x)),
             cv::Scalar(0, 255, 0), 2, cv::LINE_8);
  }
}

}  // namespace map
}  // namespace anyangle