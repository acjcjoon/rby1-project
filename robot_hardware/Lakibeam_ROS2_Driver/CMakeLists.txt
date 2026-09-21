cmake_minimum_required(VERSION 3.8)
project(lakibeam1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(CURL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${ament_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_scan_node
  src/lakibeam1_scan.cpp
  src/remote.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_scan_node
  ${ament_LIBRARIES}
  curl
)

ament_target_dependencies(
  ${PROJECT_NAME}_scan_node
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  CURL
)

install(TARGETS
  ${PROJECT_NAME}_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/scan_merger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()