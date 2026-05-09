cmake_minimum_required(VERSION 3.8)
project(tb4_openx_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Build the Navigator Library
add_library(navigator src/navigator.cpp)
target_include_directories(navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(navigator
  rclcpp rclcpp_action nav2_msgs geometry_msgs nav_msgs lifecycle_msgs action_msgs)

# Build the Patrol Executable
# add_executable(patrol_node src/patrol.cpp)
# target_link_libraries(patrol_node navigator)
# ament_target_dependencies(patrol_node rclcpp rclcpp_action nav2_msgs geometry_msgs)

# Install rules
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/mission_controller.py
  scripts/real_mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  navigator 
  # patrol_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
