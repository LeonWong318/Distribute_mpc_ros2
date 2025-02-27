cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

# Enable strict compilation warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Enable testing if requested
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install required directories for Gazebo
install(DIRECTORY
  launch
  maps
  media
  models
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Export package dependencies
ament_export_dependencies(ament_cmake)

ament_package()
