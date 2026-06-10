cmake_minimum_required(VERSION 3.8)
project(otonav_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Header-only kinematics library.
add_library(diff_drive_kinematics INTERFACE)
target_include_directories(diff_drive_kinematics INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(diff_drive_kinematics INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS diff_drive_kinematics EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_diff_drive_kinematics test/test_diff_drive_kinematics.cpp)
  target_link_libraries(test_diff_drive_kinematics diff_drive_kinematics)
endif()

ament_package()
