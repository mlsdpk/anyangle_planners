#include <memory>

#include "test_planner.hpp"

int main(int argc, char** argv) {
  ros::NodeHandle nh;
  ros::NodeHandle prv_nh("~");

  auto node = std::make_shared<TestPlanner>(nh, prv_nh);
  ros::spin();

  return 0;
}