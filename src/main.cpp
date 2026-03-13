#include "text_nav_bridge/text_nav_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<text_nav_bridge::TextNavBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
