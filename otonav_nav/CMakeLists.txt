cmake_minimum_required(VERSION 3.8)
project(otonav_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(otonav_nav_node src/go_to_goal_node.cpp)
target_include_directories(otonav_nav_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(otonav_nav_node rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS otonav_nav_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_go_to_goal test/test_go_to_goal.cpp)
  target_include_directories(test_go_to_goal PRIVATE include)
  ament_add_gtest(test_obstacle_guard test/test_obstacle_guard.cpp)
  target_include_directories(test_obstacle_guard PRIVATE include)
endif()

ament_package()
