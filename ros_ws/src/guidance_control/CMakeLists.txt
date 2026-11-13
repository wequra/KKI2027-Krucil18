cmake_minimum_required(VERSION 3.8)
project(guidance_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# cv_bridge exports the OpenCV dependency it was built against. Reuse that
# package configuration when available so direct OpenCV links do not select a
# different installation/ABI.
if(NOT DEFINED OpenCV_CONFIG_PATH OR
   NOT EXISTS "${OpenCV_CONFIG_PATH}/OpenCVConfig.cmake")
  message(FATAL_ERROR
    "cv_bridge did not export a usable OpenCV_CONFIG_PATH; refusing to select "
    "another OpenCV installation because that can mix incompatible ABIs.")
endif()
find_package(OpenCV REQUIRED CONFIG COMPONENTS core imgproc imgcodecs highgui videoio
  PATHS "${OpenCV_CONFIG_PATH}" NO_DEFAULT_PATH)
find_package(msgs REQUIRED)

include_directories(include)

add_executable(waypoint_follower_node src/waypoint_follower_node.cpp)
ament_target_dependencies(waypoint_follower_node
  rclcpp sensor_msgs geometry_msgs msgs)

add_executable(buoy_centering_node src/buoy_centering_node.cpp)
ament_target_dependencies(buoy_centering_node
  rclcpp sensor_msgs std_msgs geometry_msgs mavros_msgs cv_bridge msgs)
target_include_directories(buoy_centering_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(buoy_centering_node ${OpenCV_LIBRARIES})

add_executable(cmd_vel_to_rc_override_node src/cmd_vel_to_rc_override_node.cpp)
ament_target_dependencies(cmd_vel_to_rc_override_node
  rclcpp geometry_msgs mavros_msgs)

install(TARGETS
  waypoint_follower_node
  buoy_centering_node
  cmd_vel_to_rc_override_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/keyboard_teleop_node.py
  RENAME keyboard_teleop_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_gate_logic test/test_gate_logic.cpp)
  add_test(NAME gate_logic COMMAND test_gate_logic)
endif()

ament_package()
