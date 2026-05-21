cmake_minimum_required(VERSION 3.8)
project(log_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(csv_player src/csv_player.cpp)
ament_target_dependencies(csv_player
  rclcpp
  std_msgs
)

add_executable(log_decoder src/log_decoder.cpp)
ament_target_dependencies(log_decoder
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_include_directories(log_decoder PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(rviz_visual src/rviz_visual.cpp)
ament_target_dependencies(rviz_visual
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)
target_include_directories(rviz_visual PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(wrench_observer src/wrench_observer.cpp)
ament_target_dependencies(wrench_observer
  rclcpp
  std_msgs
  geometry_msgs
)
target_include_directories(wrench_observer PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(normal_vector_estimation src/normal_vector_estimation.cpp)
ament_target_dependencies(normal_vector_estimation
  rclcpp
  std_msgs
  geometry_msgs
)
target_include_directories(normal_vector_estimation PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(offline_result_logger src/offline_result_logger.cpp)
ament_target_dependencies(offline_result_logger
  rclcpp
  std_msgs
  geometry_msgs
)
target_include_directories(offline_result_logger PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  csv_player
  log_decoder
  rviz_visual
  wrench_observer
  normal_vector_estimation
  offline_result_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/log_player_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
