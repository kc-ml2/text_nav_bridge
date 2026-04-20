// Copyright 2026 KC-ML2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace text_nav_bridge
{

struct LandmarkEntry
{
  int id;
  std::string text;
  double x, y, z;
  double confidence;
};

// Subscribes to text commands, matches them against landmarks loaded from a
// YAML file using text similarity, ray-marches a reachable goal on the Nav2
// costmap, and dispatches NavigateToPose action goals to the Nav2 stack.
class TextNavBridgeNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit TextNavBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadLandmarks(const std::string & filepath);
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);

  int findBestLandmark(const std::string & query);

  void sendGoal(const LandmarkEntry & landmark);
  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(
    const GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);

  bool getRobotPose(double & x, double & y, double & yaw);

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // Ray-march from robot toward landmark on a local costmap snapshot, returning
  // the last free cell encountered along the line as the navigation goal.
  bool findNearestFreeGoal(
    double landmark_x, double landmark_y,
    double robot_x, double robot_y,
    double & goal_x, double & goal_y);

  void publishGoalMarker(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::string & text);

  void publishLandmarkMarkers();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_marker_pub_;
  rclcpp::TimerBase::SharedPtr landmark_marker_timer_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<LandmarkEntry> landmarks_;

  std::string landmark_file_;
  double match_threshold_;
  std::string robot_frame_;
  std::string world_frame_;

  const std::string action_server_name_ = "navigate_to_pose";
  const std::string command_topic_ = "/text_nav/command";
  const std::string status_topic_ = "/text_nav/status";
  const std::string goal_marker_topic_ = "/text_nav/goal_marker";
  const std::string landmark_marker_topic_ = "/textmap/markers";
  const std::string costmap_topic_ = "/map";

  static constexpr double marker_publish_rate_sec_ = 1.0;
  static constexpr double action_server_timeout_sec_ = 5.0;
  static constexpr double tf_lookup_timeout_sec_ = 1.0;

  static constexpr double goal_marker_color_r_ = 0.0;
  static constexpr double goal_marker_color_g_ = 1.0;
  static constexpr double goal_marker_color_b_ = 0.0;
  static constexpr double goal_marker_color_a_ = 1.0;

  static constexpr double similarity_epsilon_ = 0.01;
  // costmap cell value below this = free
  static constexpr int8_t free_cell_threshold_ = 50;
  static constexpr double goal_marker_scale_x_ = 0.5;
  static constexpr double goal_marker_scale_yz_ = 0.1;
  static constexpr double landmark_cube_size_ = 0.3;
  static constexpr double high_conf_threshold_ = 0.7;
  static constexpr double mid_conf_threshold_ = 0.4;
  static constexpr double text_z_offset_ = 0.4;
  static constexpr double text_marker_scale_ = 0.3;
  static constexpr double landmark_marker_alpha_ = 0.8;
  const std::string goal_marker_ns_ = "text_nav_goal";
  const std::string landmark_marker_ns_ = "navocr_landmarks";
  const std::string landmark_text_ns_ = "navocr_landmark_text";
};

}  // namespace text_nav_bridge
