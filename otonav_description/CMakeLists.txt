cmake_minimum_required(VERSION 3.8)
project(otonav_description)

find_package(ament_cmake REQUIRED)

# Install the MJCF model so other packages resolve it via the package share dir.
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # xmllint has no schema for MJCF; lint only well-formedness of our XML.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
