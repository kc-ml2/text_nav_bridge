#ifndef TEXT_NAV_BRIDGE__TEXT_NAV_BRIDGE_NODE_HPP_
#define TEXT_NAV_BRIDGE__TEXT_NAV_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>
#include <string>

namespace text_nav_bridge
{

struct LandmarkEntry
{
  int id;
  std::string text;
  double x, y, z;
  double confidence;
  int observation_count;
};

class TextNavBridgeNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit TextNavBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadLandmarks(const std::string & filepath);
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);

  // Find best matching landmark for query text
  int findBestLandmark(const std::string & query);

  // Nav2 goal management
  void sendGoal(const LandmarkEntry & landmark);
  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(const GoalHandleNav::SharedPtr,
                        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);

  // Get robot position from TF
  bool getRobotPosition(double & x, double & y);

  // Publish goal marker for RViz
  void publishGoalMarker(const geometry_msgs::msg::PoseStamped & goal_pose,
                         const std::string & text);

  // Subscribers / Publishers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

  // Nav2 action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Landmark data
  std::vector<LandmarkEntry> landmarks_;

  // Parameters
  std::string landmark_file_;
  double match_threshold_;
  double approach_distance_;
  std::string robot_frame_;
  std::string world_frame_;
};

}  // namespace text_nav_bridge

#endif  // TEXT_NAV_BRIDGE__TEXT_NAV_BRIDGE_NODE_HPP_
