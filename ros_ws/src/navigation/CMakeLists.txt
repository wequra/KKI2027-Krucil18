cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Belum ada node aktif — tambahkan add_executable() di sini kalau tim
# butuh state estimation tambahan (mis. EKF lokal, sensor fusion).
# Contoh:
# include_directories(include)
# add_executable(state_estimator_node src/state_estimator_node.cpp)
# ament_target_dependencies(state_estimator_node rclcpp)
# install(TARGETS state_estimator_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
