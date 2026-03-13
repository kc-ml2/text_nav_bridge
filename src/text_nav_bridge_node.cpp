#include "text_nav_bridge/text_nav_bridge_node.hpp"
#include <textmap/text_similarity.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/utils.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>

namespace text_nav_bridge
{

TextNavBridgeNode::TextNavBridgeNode(const rclcpp::NodeOptions & options)
: Node("text_nav_bridge", options)
{
  // Declare parameters
  this->declare_parameter("landmark_file", "");
  this->declare_parameter("match_threshold", 0.5);
  this->declare_parameter("approach_distance", 1.0);
  this->declare_parameter("robot_frame", "camera_link");
  this->declare_parameter("world_frame", "map");

  // Get parameters
  landmark_file_ = this->get_parameter("landmark_file").as_string();
  match_threshold_ = this->get_parameter("match_threshold").as_double();
  approach_distance_ = this->get_parameter("approach_distance").as_double();
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  world_frame_ = this->get_parameter("world_frame").as_string();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load landmarks
  if (landmark_file_.empty()) {
    RCLCPP_FATAL(this->get_logger(), "landmark_file parameter is not set. Shutting down.");
    rclcpp::shutdown();
    return;
  }
  loadLandmarks(landmark_file_);

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Subscribers
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/text_nav/command", 10,
    std::bind(&TextNavBridgeNode::commandCallback, this, std::placeholders::_1));

  // Publishers
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/text_nav/status", 10);
  goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/text_nav/goal_marker", 10);

  RCLCPP_INFO(this->get_logger(), "TextNavBridge started with %zu landmarks loaded",
              landmarks_.size());
  RCLCPP_INFO(this->get_logger(), "  Match threshold: %.2f", match_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Approach distance: %.1fm", approach_distance_);
}

void TextNavBridgeNode::loadLandmarks(const std::string & filepath)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(filepath);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to parse YAML file '%s': %s",
                 filepath.c_str(), e.what());
    rclcpp::shutdown();
    return;
  }

  if (!config["landmarks"]) {
    RCLCPP_WARN(this->get_logger(), "No 'landmarks' key found in %s", filepath.c_str());
    return;
  }

  for (const auto & node : config["landmarks"]) {
    LandmarkEntry entry;
    entry.id = node["id"].as<int>();
    entry.text = node["text"].as<std::string>();
    entry.x = node["position"]["x"].as<double>();
    entry.y = node["position"]["y"].as<double>();
    entry.z = node["position"]["z"].as<double>();
    entry.confidence = node["confidence"].as<double>();
    entry.observation_count = node["observation_count"].as<int>();
    landmarks_.push_back(entry);

    RCLCPP_INFO(this->get_logger(), "  Loaded landmark #%d: '%s' at (%.2f, %.2f, %.2f)",
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
      if (i > 0) status.data += ", ";
      status.data += landmarks_[i].text;
    }
    status_pub_->publish(status);
    RCLCPP_WARN(this->get_logger(), "%s", status.data.c_str());
    return;
  }

  // Cancel previous goal if active
  if (current_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Cancelling previous navigation goal");
    nav_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_ = nullptr;
  }

  RCLCPP_INFO(this->get_logger(), "Best match: '%s' (id=%d) at (%.2f, %.2f)",
              landmarks_[best_idx].text.c_str(), landmarks_[best_idx].id,
              landmarks_[best_idx].x, landmarks_[best_idx].y);

  sendGoal(landmarks_[best_idx]);
}

int TextNavBridgeNode::findBestLandmark(const std::string & query)
{
  int best_idx = -1;
  double best_sim = 0.0;
  double best_dist = std::numeric_limits<double>::max();

  double robot_x = 0.0, robot_y = 0.0;
  bool has_robot_pos = getRobotPosition(robot_x, robot_y);

  for (size_t i = 0; i < landmarks_.size(); i++) {
    double sim = text_utils::textSimilarity(query, landmarks_[i].text);
    if (sim < match_threshold_) continue;

    if (sim > best_sim) {
      best_sim = sim;
      best_idx = static_cast<int>(i);
      if (has_robot_pos) {
        double dx = landmarks_[i].x - robot_x;
        double dy = landmarks_[i].y - robot_y;
        best_dist = std::sqrt(dx * dx + dy * dy);
      }
    } else if (std::abs(sim - best_sim) < 0.01 && has_robot_pos) {
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
    RCLCPP_INFO(this->get_logger(), "Match: '%s' → '%s' (similarity=%.2f)",
                query.c_str(), landmarks_[best_idx].text.c_str(), best_sim);
  }

  return best_idx;
}

void TextNavBridgeNode::sendGoal(const LandmarkEntry & landmark)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
    auto status = std_msgs::msg::String();
    status.data = "ERROR: Nav2 action server not available";
    status_pub_->publish(status);
    return;
  }

  // Compute goal pose: approach_distance_ meters in front of landmark
  double robot_x = 0.0, robot_y = 0.0;
  if (!getRobotPosition(robot_x, robot_y)) {
    RCLCPP_ERROR(this->get_logger(), "Cannot get robot position from TF");
    auto status = std_msgs::msg::String();
    status.data = "ERROR: Cannot get robot position";
    status_pub_->publish(status);
    return;
  }

  // Direction from robot to landmark
  double dx = landmark.x - robot_x;
  double dy = landmark.y - robot_y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double yaw = std::atan2(dy, dx);

  // Goal position: approach_distance_ before the landmark
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = world_frame_;
  goal_pose.header.stamp = this->now();

  if (dist > approach_distance_) {
    goal_pose.pose.position.x = landmark.x - approach_distance_ * (dx / dist);
    goal_pose.pose.position.y = landmark.y - approach_distance_ * (dy / dist);
  } else {
    // Already close enough, just face the landmark
    goal_pose.pose.position.x = robot_x;
    goal_pose.pose.position.y = robot_y;
  }
  goal_pose.pose.position.z = 0.0;

  // Orientation: face the landmark
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.0;
  goal_pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_pose.pose.orientation.w = std::cos(yaw / 2.0);

  // Send goal
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TextNavBridgeNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&TextNavBridgeNode::feedbackCallback, this,
              std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&TextNavBridgeNode::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(this->get_logger(),
              "Sent goal to '%s': (%.2f, %.2f), yaw=%.1f°, distance=%.1fm",
              landmark.text.c_str(), goal_pose.pose.position.x,
              goal_pose.pose.position.y, yaw * 180.0 / M_PI, dist);

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

bool TextNavBridgeNode::getRobotPosition(double & x, double & y)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      world_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

void TextNavBridgeNode::publishGoalMarker(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & text)
{
  visualization_msgs::msg::Marker marker;
  marker.header = goal_pose.header;
  marker.ns = "text_nav_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = goal_pose.pose;
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration(0, 0);  // Persistent
  marker.text = text;
  goal_marker_pub_->publish(marker);
}

}  // namespace text_nav_bridge
