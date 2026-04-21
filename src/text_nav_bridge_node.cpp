// Copyright 2026 KC-ML2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "text_nav_bridge/text_nav_bridge_node.hpp"

#include <tf2/utils.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <textmap/text_similarity.hpp>

namespace text_nav_bridge
{

TextNavBridgeNode::TextNavBridgeNode(const rclcpp::NodeOptions & options)
: Node("text_nav_bridge", options)
{
  this->declare_parameter("landmark_file", "");
  this->declare_parameter("match_threshold", 0.5);
  this->declare_parameter("robot_frame", "camera_link");
  this->declare_parameter("world_frame", "map");

  landmark_file_ = this->get_parameter("landmark_file").as_string();
  match_threshold_ = this->get_parameter("match_threshold").as_double();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  world_frame_ = this->get_parameter("world_frame").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (landmark_file_.empty()) {
    RCLCPP_FATAL(this->get_logger(), "landmark_file parameter is not set. Shutting down.");
    rclcpp::shutdown();
    return;
  }
  loadLandmarks(landmark_file_);

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_server_name_);

  // One-shot availability check at startup so the command callback never blocks the executor.
  if (!nav_client_->wait_for_action_server(
      std::chrono::duration<double>(action_server_timeout_sec_)))
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Nav2 action server '%s' not available after %.1fs — "
      "commands received before it comes up will be rejected.",
      action_server_name_.c_str(), action_server_timeout_sec_);
  }

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    command_topic_, 10,
    std::bind(&TextNavBridgeNode::commandCallback, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&TextNavBridgeNode::costmapCallback, this, std::placeholders::_1));

  status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);
  goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    goal_marker_topic_, 10);
  landmark_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    landmark_marker_topic_, 10);

  landmark_marker_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(marker_publish_rate_sec_),
    std::bind(&TextNavBridgeNode::publishLandmarkMarkers, this));

  RCLCPP_INFO(
    this->get_logger(), "TextNavBridge started with %zu landmarks loaded",
    landmarks_.size());
  RCLCPP_INFO(this->get_logger(), "  Match threshold: %.2f", match_threshold_);
}

void TextNavBridgeNode::loadLandmarks(const std::string & filepath)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(filepath);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to parse YAML file '%s': %s",
      filepath.c_str(), e.what());
    rclcpp::shutdown();
    return;
  }

  if (!config["landmarks"]) {
    RCLCPP_WARN(this->get_logger(), "No 'landmarks' key found in %s", filepath.c_str());
    return;
  }

  landmarks_.reserve(config["landmarks"].size());
  for (const auto & node : config["landmarks"]) {
    LandmarkEntry entry;
    entry.id = node["id"].as<int>();
    entry.text = node["text"].as<std::string>();
    entry.x = node["position"]["x"].as<double>();
    entry.y = node["position"]["y"].as<double>();
    entry.z = node["position"]["z"].as<double>();
    entry.confidence = node["confidence"].as<double>();
    landmarks_.push_back(entry);

    RCLCPP_INFO(
      this->get_logger(), "  Loaded landmark #%d: '%s' at (%.2f, %.2f, %.2f)",
      entry.id, entry.text.c_str(), entry.x, entry.y, entry.z);
  }
}

void TextNavBridgeNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string query = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received navigation command: '%s'", query.c_str());

  if (landmarks_.empty()) {
    auto status = std_msgs::msg::String();
    status.data = "ERROR: No landmarks loaded";
    status_pub_->publish(status);
    RCLCPP_ERROR(this->get_logger(), "No landmarks loaded");
    return;
  }

  int best_idx = findBestLandmark(query);
  if (best_idx < 0) {
    auto status = std_msgs::msg::String();
    status.data = "ERROR: No matching landmark found for '" + query + "'. Available: ";
    for (size_t i = 0; i < landmarks_.size(); i++) {
      if (i > 0) {status.data += ", ";}
      status.data += landmarks_[i].text;
    }
    status_pub_->publish(status);
    RCLCPP_WARN(this->get_logger(), "%s", status.data.c_str());
    return;
  }

  if (current_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Cancelling previous navigation goal");
    nav_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_ = nullptr;
  }

  RCLCPP_INFO(
    this->get_logger(), "Best match: '%s' (id=%d) at (%.2f, %.2f)",
    landmarks_[best_idx].text.c_str(), landmarks_[best_idx].id,
    landmarks_[best_idx].x, landmarks_[best_idx].y);

  sendGoal(landmarks_[best_idx]);
}

int TextNavBridgeNode::findBestLandmark(const std::string & query)
{
  int best_idx = -1;
  double best_sim = 0.0;
  double best_dist = std::numeric_limits<double>::max();

  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  bool has_robot_pos = getRobotPose(robot_x, robot_y, robot_yaw);

  for (size_t i = 0; i < landmarks_.size(); i++) {
    double sim = text_utils::textSimilarity(query, landmarks_[i].text);
    if (sim < match_threshold_) {continue;}

    if (sim > best_sim) {
      best_sim = sim;
      best_idx = static_cast<int>(i);
      if (has_robot_pos) {
        double dx = landmarks_[i].x - robot_x;
        double dy = landmarks_[i].y - robot_y;
        best_dist = std::sqrt(dx * dx + dy * dy);
      }
    } else if (std::abs(sim - best_sim) < similarity_epsilon_ && has_robot_pos) {
      // Same text similarity — pick the closer one
      double dx = landmarks_[i].x - robot_x;
      double dy = landmarks_[i].y - robot_y;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < best_dist) {
        best_dist = d;
        best_idx = static_cast<int>(i);
      }
    }
  }

  if (best_idx >= 0) {
    RCLCPP_INFO(
      this->get_logger(), "Match: '%s' -> '%s' (similarity=%.2f)",
      query.c_str(), landmarks_[best_idx].text.c_str(), best_sim);
  }

  return best_idx;
}

void TextNavBridgeNode::sendGoal(const LandmarkEntry & landmark)
{
  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
    auto status = std_msgs::msg::String();
    status.data = "ERROR: Nav2 action server not available";
    status_pub_->publish(status);
    return;
  }

  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get robot position from TF");
    auto status = std_msgs::msg::String();
    status.data = "ERROR: Cannot get robot position";
    status_pub_->publish(status);
    return;
  }

  double goal_x, goal_y;
  if (!findNearestFreeGoal(landmark.x, landmark.y, robot_x, robot_y, goal_x, goal_y)) {
    RCLCPP_ERROR(this->get_logger(), "Costmap not yet received, cannot compute goal");
    auto status = std_msgs::msg::String();
    status.data = "ERROR: Costmap not yet received";
    status_pub_->publish(status);
    return;
  }

  double yaw = std::atan2(landmark.y - goal_y, landmark.x - goal_x);

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = world_frame_;
  goal_pose.header.stamp = this->now();
  goal_pose.pose.position.x = goal_x;
  goal_pose.pose.position.y = goal_y;
  goal_pose.pose.position.z = 0.0;

  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.0;
  goal_pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_pose.pose.orientation.w = std::cos(yaw / 2.0);

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TextNavBridgeNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &TextNavBridgeNode::feedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&TextNavBridgeNode::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);

  double goal_dist = std::sqrt(std::pow(goal_x - robot_x, 2) + std::pow(goal_y - robot_y, 2));
  RCLCPP_INFO(
    this->get_logger(),
    "Sent goal to '%s': (%.2f, %.2f), yaw=%.1fdeg, distance=%.1fm",
    landmark.text.c_str(), goal_pose.pose.position.x,
    goal_pose.pose.position.y, yaw * 180.0 / M_PI, goal_dist);

  publishGoalMarker(goal_pose, landmark.text);

  auto status = std_msgs::msg::String();
  status.data = "NAVIGATING: heading to '" + landmark.text + "'";
  status_pub_->publish(status);
}

void TextNavBridgeNode::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
    auto status = std_msgs::msg::String();
    status.data = "ERROR: Goal rejected by Nav2";
    status_pub_->publish(status);
    return;
  }
  current_goal_handle_ = goal_handle;
  RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
}

void TextNavBridgeNode::feedbackCallback(
  const GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  auto status = std_msgs::msg::String();
  status.data = "NAVIGATING: distance_remaining=" +
    std::to_string(feedback->distance_remaining) + "m";
  status_pub_->publish(status);
}

void TextNavBridgeNode::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  current_goal_handle_ = nullptr;

  auto status = std_msgs::msg::String();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      status.data = "SUCCESS: Navigation completed";
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      status.data = "FAILED: Navigation aborted";
      RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      status.data = "CANCELED: Navigation canceled";
      RCLCPP_WARN(this->get_logger(), "Navigation canceled");
      break;
    default:
      status.data = "UNKNOWN: Navigation result unknown";
      RCLCPP_ERROR(this->get_logger(), "Unknown navigation result");
      break;
  }
  status_pub_->publish(status);
}

bool TextNavBridgeNode::getRobotPose(double & x, double & y, double & yaw)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      world_frame_, robot_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    yaw = tf2::getYaw(transform.transform.rotation);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

void TextNavBridgeNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

bool TextNavBridgeNode::findNearestFreeGoal(
  double landmark_x, double landmark_y,
  double robot_x, double robot_y,
  double & goal_x, double & goal_y)
{
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap = latest_costmap_;
  }
  if (!costmap) {
    return false;
  }

  const auto & info = costmap->info;
  const auto & data = costmap->data;
  double resolution = info.resolution;
  double origin_x = info.origin.position.x;
  double origin_y = info.origin.position.y;

  double dx = landmark_x - robot_x;
  double dy = landmark_y - robot_y;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 1e-6) {
    goal_x = robot_x;
    goal_y = robot_y;
    return true;
  }
  double step_x = dx / dist * resolution;
  double step_y = dy / dist * resolution;

  int max_steps = static_cast<int>(dist / resolution) + 1;
  double cx = robot_x;
  double cy = robot_y;
  bool found = false;

  for (int i = 0; i <= max_steps; i++) {
    int mx = static_cast<int>((cx - origin_x) / resolution);
    int my = static_cast<int>((cy - origin_y) / resolution);

    if (mx >= 0 && mx < static_cast<int>(info.width) &&
      my >= 0 && my < static_cast<int>(info.height))
    {
      int idx = my * info.width + mx;
      int8_t cost = data[idx];
      if (cost >= 0 && cost < free_cell_threshold_) {
        goal_x = cx;
        goal_y = cy;
        found = true;
      }
    }

    cx += step_x;
    cy += step_y;
  }

  if (found) {
    RCLCPP_INFO(
      this->get_logger(),
      "Found free cell at (%.2f, %.2f), %.1fm from landmark",
      goal_x, goal_y,
      std::sqrt(std::pow(goal_x - landmark_x, 2) + std::pow(goal_y - landmark_y, 2)));
    return true;
  }

  goal_x = robot_x;
  goal_y = robot_y;
  RCLCPP_WARN(this->get_logger(), "No free cell found, falling back to robot position");
  return true;
}

void TextNavBridgeNode::publishGoalMarker(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & text)
{
  if (goal_marker_pub_->get_subscription_count() == 0) {
    return;
  }

  auto marker = std::make_unique<visualization_msgs::msg::Marker>();
  marker->header = goal_pose.header;
  marker->ns = goal_marker_ns_;
  marker->id = 0;
  marker->type = visualization_msgs::msg::Marker::ARROW;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->pose = goal_pose.pose;
  marker->scale.x = goal_marker_scale_x_;
  marker->scale.y = goal_marker_scale_yz_;
  marker->scale.z = goal_marker_scale_yz_;
  marker->color.r = goal_marker_color_r_;
  marker->color.g = goal_marker_color_g_;
  marker->color.b = goal_marker_color_b_;
  marker->color.a = goal_marker_color_a_;
  marker->lifetime = rclcpp::Duration(0, 0);
  marker->text = text;
  goal_marker_pub_->publish(std::move(marker));
}

void TextNavBridgeNode::publishLandmarkMarkers()
{
  if (landmark_marker_pub_->get_subscription_count() == 0) {
    return;
  }

  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  marker_array->markers.reserve(landmarks_.size() * 2);
  int marker_id = 0;

  for (const auto & lm : landmarks_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = this->now();
    marker.ns = landmark_marker_ns_;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = lm.x;
    marker.pose.position.y = lm.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = landmark_cube_size_;
    marker.scale.y = landmark_cube_size_;
    marker.scale.z = landmark_cube_size_;

    if (lm.confidence >= high_conf_threshold_) {
      marker.color.r = 0.0; marker.color.g = 0.8; marker.color.b = 0.0;
    } else if (lm.confidence >= mid_conf_threshold_) {
      marker.color.r = 1.0; marker.color.g = 0.8; marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0; marker.color.g = 0.4; marker.color.b = 0.0;
    }
    marker.color.a = landmark_marker_alpha_;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker_array->markers.push_back(marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = landmark_text_ns_;
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose = marker.pose;
    text_marker.pose.position.z = text_z_offset_;
    text_marker.text = lm.text;
    text_marker.scale.z = text_marker_scale_;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = rclcpp::Duration(0, 0);
    marker_array->markers.push_back(text_marker);
  }

  landmark_marker_pub_->publish(std::move(marker_array));
}

}  // namespace text_nav_bridge
