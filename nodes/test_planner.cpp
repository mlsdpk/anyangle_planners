#include "test_planner.hpp"

TestPlanner::TestPlanner(ros::NodeHandle& nh, ros::NodeHandle& prv_nh) {
  std::string map_file;
  prv_nh.param<std::string>("map_file", map_file, std::string(""));

  map_loader_ = anyangle::map::MapLoaderPtr(
      map_file, anyangle::map::EnvironmentType::IMAGE);

  start_state_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "initialpose", 1, &TestPlanner::startStateCallback, this);
  goal_state_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 1, &TestPlanner::goalStateCallback, this);

  map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  markers_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);

  nav_msgs::OccupancyGrid ogm;
  map_loader_->toOccupancyGrid(ogm);
  map_pub_.publish(ogm);
}

void TestPlanner::startStateCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_state) {
  // check collision in the map
  if (map_loader_->getEnv()->inCollision(start_state->pose.pose.position.x,
                                         start_state->pose.pose.position.y)) {
    ROS_ERROR("Given start state (x: %f, y: %f) is in collision. Rejected.",
              start_state->pose.pose.position.x,
              start_state->pose.pose.position.y);
    return;
  }

  ROS_INFO("Provided start state (x: %f, y: %f).",
           start_state->pose.pose.position.x,
           start_state->pose.pose.position.y);

  start_state_ = start_state->pose.pose;
  start_state_exists_ = true;
}

void TestPlanner::goalStateCallback(
    const geometry_msgs::PoseStamped::ConstPtr& goal_state) {
  if (!start_state_exists_) {
    ROS_ERROR(
        "Start state is not available. Please provide state state first.");
    return;
  }

  // check collision in the map
  if (map_loader_->getEnv()->inCollision(goal_state->pose.pose.position.x,
                                         goal_state->pose.pose.position.y)) {
    ROS_ERROR("Given goal state (x: %f, y: %f) is in collision. Rejected.",
              goal_state->pose.pose.position.x,
              goal_state->pose.pose.position.y);
    return;
  }

  ROS_INFO("Provided goal state (x: %f, y: %f).",
           goal_state->pose.pose.position.x, goal_state->pose.pose.position.y);

  goal_state_ = goal_state->pose;

  // publish markers for visualization purposes
  {
    // start state
    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "start_state";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.position.x = start_state_.position.x;
    start_marker.pose.position.y = start_state_.position.y;
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = 0.1;
    start_marker.scale.y = 0.1;
    start_marker.scale.z = 0.0;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;

    // goal state
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "goal_state";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = goal_state_.position.x;
    goal_marker.pose.position.y = goal_state_.position.y;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.1;
    goal_marker.scale.y = 0.1;
    goal_marker.scale.z = 0.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(std::move(start_marker));
    markers.markers.push_back(std::move(goal_marker));

    markers_pub_.publish(markers);
  }
}