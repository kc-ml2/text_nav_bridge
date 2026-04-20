// Copyright 2026 KC-ML2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "text_nav_bridge/text_nav_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<text_nav_bridge::TextNavBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
