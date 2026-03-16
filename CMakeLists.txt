cmake_minimum_required(VERSION 3.8)
project(text_nav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(textmap REQUIRED)

# Include directories
include_directories(include)

# text_nav_bridge node
add_executable(text_nav_bridge_node
  src/text_nav_bridge_node.cpp
  src/main.cpp
)

ament_target_dependencies(text_nav_bridge_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  visualization_msgs
  nav2_msgs
  tf2
  tf2_ros
  textmap
)

target_link_libraries(text_nav_bridge_node yaml-cpp)

# Install executables
install(TARGETS
  text_nav_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
