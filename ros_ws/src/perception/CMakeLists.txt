cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

add_executable(vision_detector_node src/vision_detector_node.cpp)
ament_target_dependencies(vision_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge msgs)
target_include_directories(vision_detector_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_detector_node ${OpenCV_LIBRARIES})

add_executable(hsv_tuner_node src/hsv_tuner_node.cpp)
ament_target_dependencies(hsv_tuner_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge msgs)
target_include_directories(hsv_tuner_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hsv_tuner_node ${OpenCV_LIBRARIES})

add_executable(opencv_camera_node src/opencv_camera_node.cpp)
ament_target_dependencies(opencv_camera_node
  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge msgs)
target_include_directories(opencv_camera_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv_camera_node ${OpenCV_LIBRARIES})

install(TARGETS vision_detector_node hsv_tuner_node opencv_camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/camera_rtsp_streamer_node.py
  scripts/ball_detector_node.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  weights/best.pt
  DESTINATION share/${PROJECT_NAME}/weights)

ament_package()
