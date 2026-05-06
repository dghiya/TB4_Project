cmake_minimum_required(VERSION 3.8)
project(tb4_openx_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tb4_openx_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# 1. Approach Server
add_executable(approach_trash_server src/ApproachTrash.cpp src/approach_node.cpp)
target_include_directories(approach_trash_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(approach_trash_server
  rclcpp rclcpp_action moveit_ros_planning_interface tb4_openx_interfaces tf2_ros tf2_geometry_msgs geometry_msgs nav2_msgs)

# 2. Pick Server
add_executable(pick_trash_server src/PickTrash.cpp src/pick_node.cpp)
target_include_directories(pick_trash_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pick_trash_server
  rclcpp rclcpp_action moveit_ros_planning_interface tb4_openx_interfaces tf2_ros tf2_geometry_msgs geometry_msgs)

# 3. Place Server
add_executable(place_trash_server src/PlaceTrash.cpp src/place_node.cpp)
target_include_directories(place_trash_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(place_trash_server
  rclcpp rclcpp_action moveit_ros_planning_interface tb4_openx_interfaces tf2_ros tf2_geometry_msgs geometry_msgs nav2_msgs)

# Install Rules
install(TARGETS
  approach_trash_server
  pick_trash_server
  place_trash_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/trash_collection_task.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()