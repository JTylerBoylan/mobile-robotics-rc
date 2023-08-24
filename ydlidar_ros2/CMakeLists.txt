cmake_minimum_required(VERSION 3.8)
project(ydlidar_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ydlidar_sdk REQUIRED)

include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

add_executable(ydlidar_ros2_node src/ydlidar_ros2_node.cpp)
ament_target_dependencies(ydlidar_ros2_node rclcpp rclcpp_lifecycle sensor_msgs)
target_link_libraries(ydlidar_ros2_node ${YDLIDAR_SDK_LIBRARIES})

install(TARGETS 
  ydlidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
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
