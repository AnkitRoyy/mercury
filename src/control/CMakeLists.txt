cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(nav_msgs         REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(nav2_core        REQUIRED)
find_package(nav2_costmap_2d  REQUIRED)
find_package(nav2_util        REQUIRED)
find_package(pluginlib         REQUIRED)
find_package(tf2_ros           REQUIRED)

# ── Stanley controller plugin ─────────────────────────────────────────────────
add_library(stanley_controller SHARED
  src/stanley_controller.cpp
)

target_include_directories(stanley_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stanley_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# ── install ───────────────────────────────────────────────────────────────────
install(TARGETS stanley_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# ── testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND    TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()