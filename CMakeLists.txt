cmake_minimum_required(VERSION 3.8)
project(text_nav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(text_nav_bridge_node
  src/text_nav_bridge_node.cpp
  src/main.cpp
)

target_include_directories(text_nav_bridge_node PRIVATE
  include
)

ament_target_dependencies(text_nav_bridge_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
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
