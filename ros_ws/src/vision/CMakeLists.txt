cmake_minimum_required(VERSION 3.8)
project(vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs REQUIRED)

include_directories(include)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_node rclcpp msgs)

install(TARGETS obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
