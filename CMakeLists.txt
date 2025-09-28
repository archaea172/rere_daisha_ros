cmake_minimum_required(VERSION 3.8)
project(rere_daisha_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(ldlidar_node REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS scripts/yolo_test.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(ransac_ball
  src/ransac_ball.cpp
)

target_include_directories(ransac_ball PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(debug_node
  src/debug.cpp
)

install(
  TARGETS debug_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(debug_node
  ransac_ball
  ${OpenCV_LIBRARIES}
)

add_executable(ransac_ball_node
  src/ransac_ball_node.cpp
  include/${PROJECT_NAME}/ransac_ball_node.hpp
)

target_link_libraries(ransac_ball_node
  ransac_ball
)

ament_target_dependencies(ransac_ball_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
)

install(
  TARGETS ransac_ball_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY weights images launch config
  DESTINATION share/${PROJECT_NAME}
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
