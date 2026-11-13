cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/synthetic_buoy_camera.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(full_mission_stress_test test/test_full_mission_stress.py TIMEOUT 180)
  ament_add_pytest_test(buoy_target_locking_test test/test_buoy_target_locking.py TIMEOUT 120)
  ament_add_pytest_test(tracked_gate_lifecycle_test test/test_tracked_gate_lifecycle.py TIMEOUT 180)
  ament_add_pytest_test(tracker_persistence_test test/test_tracker_persistence.py TIMEOUT 180)
  set_tests_properties(full_mission_stress_test buoy_target_locking_test tracked_gate_lifecycle_test
    PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
