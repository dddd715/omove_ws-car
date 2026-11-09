cmake_minimum_required(VERSION 3.8)
project(omove_description)
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
add_library(visited_trail_layer SHARED src/visited_trail_layer.cpp)
target_include_directories(visited_trail_layer PRIVATE include)
target_compile_features(visited_trail_layer PUBLIC cxx_std_14)
ament_target_dependencies(visited_trail_layer nav2_costmap_2d pluginlib rclcpp
  rclcpp_lifecycle nav_msgs std_srvs)
pluginlib_export_plugin_description_file(nav2_costmap_2d visited_trail_plugin.xml)
install(TARGETS visited_trail_layer LIBRARY DESTINATION lib)
install(DIRECTORY urdf launch rviz worlds config maps
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  omove_description/mecanum_kinematics.py
  omove_description/mecanum_odometry.py
  omove_description/scan_to_cloud.py
  omove_description/cloud_accumulator.py
  scripts/validate_navigation.py
  scripts/validate_odometry.py
  DESTINATION lib/${PROJECT_NAME})
include(CTest)
if(BUILD_TESTING)
  find_package(nav2_util REQUIRED)
  find_package(nav2_navfn_planner REQUIRED)
  add_executable(test_visited_route test/test_visited_route.cpp)
  target_include_directories(test_visited_route PRIVATE include)
  ament_target_dependencies(test_visited_route nav2_navfn_planner)
  add_test(NAME visited_route_planning COMMAND test_visited_route)
  add_executable(test_visited_trail test/test_visited_trail.cpp)
  target_include_directories(test_visited_trail PRIVATE include)
  ament_target_dependencies(test_visited_trail nav2_costmap_2d pluginlib rclcpp nav2_util)
  add_test(NAME visited_trail_integration COMMAND test_visited_trail)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME mecanum_unit_tests
    COMMAND ${CMAKE_COMMAND} -E env PYTHONDONTWRITEBYTECODE=1
      ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -v)
endif()
ament_package()

