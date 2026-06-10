cmake_minimum_required(VERSION 3.8)
project(otonav_mujoco_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(otonav_control REQUIRED)

# --- MuJoCo C SDK discovery (the ONLY package that links MuJoCo, Hard Rule #4).
# Root comes from the MUJOCO_DIR env/cache var (set once in docker/Dockerfile).
set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "MuJoCo SDK root")
find_path(MUJOCO_INCLUDE_DIR NAMES mujoco/mujoco.h HINTS "${MUJOCO_DIR}/include")
find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco HINTS "${MUJOCO_DIR}/lib")
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo SDK not found. Set MUJOCO_DIR to the SDK root "
    "(expects <root>/include/mujoco/mujoco.h and <root>/lib/libmujoco.so). "
    "Got MUJOCO_DIR='${MUJOCO_DIR}'.")
endif()

add_executable(otonav_bridge_node
  src/bridge_node.cpp
  src/mujoco_interface.cpp
  src/can_bus_interface.cpp)
target_include_directories(otonav_bridge_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "${MUJOCO_INCLUDE_DIR}")
target_link_libraries(otonav_bridge_node "${MUJOCO_LIBRARY}")
set_target_properties(otonav_bridge_node PROPERTIES
  INSTALL_RPATH "${MUJOCO_DIR}/lib"
  BUILD_WITH_INSTALL_RPATH FALSE)
ament_target_dependencies(otonav_bridge_node
  rclcpp ament_index_cpp geometry_msgs nav_msgs sensor_msgs rosgraph_msgs otonav_control)

install(TARGETS otonav_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
