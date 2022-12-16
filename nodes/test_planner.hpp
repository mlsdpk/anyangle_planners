#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

#include "anyangle_planners/map/map_loader.hpp"

class TestPlanner {
 public:
  explicit TestPlanner(ros::NodeHandle& nh, ros::NodeHandle& prv_nh);
  ~TestPlanner() = default;

 private:
  void startStateCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_state);

  void goalStateCallback(
      const geometry_msgs::PoseStamped::ConstPtr& goal_state);

  ros::Publisher map_pub_;
  ros::Publisher markers_pub_;
  ros::Subscriber start_state_sub_;
  ros::Subscriber goal_state_sub_;

  geometry_msgs::Pose start_state_;
  geometry_msgs::Pose goal_state_;

  anyangle::map::MapLoaderPtr map_loader_;

  bool start_state_exists_{false};
};