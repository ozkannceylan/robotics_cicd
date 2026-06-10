cmake_minimum_required(VERSION 3.8)
project(otonav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Headless SIL smoke test (Phase 2): asserts the bridge publishes its core
  # topics. ROS_LOCALHOST_ONLY keeps DDS on loopback in CI (ADR-3).
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_odom_smoke.py
    TIMEOUT 120
    ENV ROS_LOCALHOST_ONLY=1)
endif()

ament_package()
