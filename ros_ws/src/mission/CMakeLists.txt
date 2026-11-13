cmake_minimum_required(VERSION 3.8)
project(mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs REQUIRED)

include_directories(include)

add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(mission_manager_node
  rclcpp std_srvs std_msgs msgs)

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/mission_controller_node.py
  scripts/mission_cmd_mux_node.py
  scripts/gcs_bridge_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
