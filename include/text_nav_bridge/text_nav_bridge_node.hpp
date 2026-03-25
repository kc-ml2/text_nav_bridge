#ifndef TEXT_NAV_BRIDGE__TEXT_NAV_BRIDGE_NODE_HPP_
#define TEXT_NAV_BRIDGE__TEXT_NAV_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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

  // Publish loaded landmarks as markers for RViz
  void publishLandmarkMarkers();

  // Subscribers / Publishers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_marker_pub_;
  rclcpp::TimerBase::SharedPtr landmark_marker_timer_;

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

  // Topic / action names
  const std::string action_server_name_ = "navigate_to_pose";
  const std::string command_topic_ = "/text_nav/command";
  const std::string status_topic_ = "/text_nav/status";
  const std::string goal_marker_topic_ = "/text_nav/goal_marker";
  const std::string landmark_marker_topic_ = "/textmap/markers";

  // Timing constants
  static constexpr double marker_publish_rate_sec_ = 1.0;
  static constexpr double action_server_timeout_sec_ = 5.0;
  static constexpr double tf_lookup_timeout_sec_ = 1.0;

  // Goal marker color (green)
  static constexpr double goal_marker_color_r_ = 0.0;
  static constexpr double goal_marker_color_g_ = 1.0;
  static constexpr double goal_marker_color_b_ = 0.0;
  static constexpr double goal_marker_color_a_ = 1.0;

  // Marker constants
  static constexpr double similarity_epsilon_ = 0.01;
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

#endif  // TEXT_NAV_BRIDGE__TEXT_NAV_BRIDGE_NODE_HPP_
