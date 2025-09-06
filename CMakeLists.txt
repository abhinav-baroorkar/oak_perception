cmake_minimum_required(VERSION 3.8)
project(oak_perception)

# Warnings + C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# --- Targets
add_executable(depth_localizer src/depth_localizer.cpp)
ament_target_dependencies(depth_localizer
  rclcpp
  sensor_msgs
  vision_msgs
  image_geometry
  cv_bridge
  message_filters
)

install(TARGETS depth_localizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (so `ros2 launch` works)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing/Lint (must be BEFORE ament_package)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip until you add headers/licenses or put package under git
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Package footer
ament_package()
