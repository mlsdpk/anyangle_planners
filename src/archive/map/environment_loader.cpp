// MIT License

// Copyright (c) 2023 Phone Thiha Kyaw

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

#include "anyangle_planners/map/environment_loader.hpp"

anyangle::map::EnvironmentPtr anyangle::map::create(const moving_ai_lab::Scenario& scenario,
                                                    EnvironmentType env_type)
{
  // check the scenario version is supported or not
  if (scenario.version != "1")
  {
    std::cerr << "Only version 1 is currently supported for Moving AI Lab scenario.\n";
    exit(0);
  }

  // must contain at least one experiment
  if (scenario.experiments.empty())
  {
    std::cerr << "Scenario must contain at least one experiment.\n";
    exit(0);
  }

  // here we assume that all the experiments inside scenario share the same map name
  // so we only use the first experiment map name
  std::string map_name = scenario.experiments.front().map_name;

  std::ifstream map_file(map_name);
  if (map_file.fail() && scenario.name.find('/') != std::string::npos)
  {
    map_name = scenario.name.substr(0, scenario.name.find_last_of('/')) + "/" + map_name;
  }

  std::cout << "Loading Moving AI Lab scenario map from " << map_name << std::endl;
  map_file = std::ifstream(map_name);

  if (map_file.fail())
  {
    std::cerr << "Failed to load map file from " << map_name << std::endl;
    exit(0);
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

  for (std::string line; std::getline(map_file, line);)
  {
    for (std::string::iterator it = line.begin(); it != line.end(); ++it)
    {
      if ((*it) == '.')
      {
        temp_map.push_back(true);
      }
      else if ((*it) == '@')
      {
        temp_map.push_back(false);
      }
    }
  }

  switch (env_type)
  {
    case EnvironmentType::GridMap:
    {
      auto env = std::make_shared<anyangle::map::GridMapEnvironment>(
          GridMapEnvironment::Connectivity::FOUR, "layer");

      // initialize gridmap
      grid_map::Length length(height, width);
      map_.setGeometry(length, 1.0, grid_map::Position(0.0, 0.0));

      map_.add("layer");
      grid_map::Matrix& data = map_["layer"];

      std::size_t i = 0u;
      for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
      {
        const grid_map::Index gridMapIndex = *iterator;
        const grid_map::Index imageIndex = iterator.getUnwrappedIndex();
        data(gridMapIndex(1), gridMapIndex(0)) = temp_map[i] ? 1.0 : 0.0;
        i++;
      }

      return env;
    }

    default:
    {
        std::cerr << "Only GridMap environment type is supported.\n";
        exit(0);
    }
  }
}