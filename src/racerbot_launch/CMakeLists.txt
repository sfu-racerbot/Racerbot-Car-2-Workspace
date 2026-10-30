cmake_minimum_required(VERSION 3.8)
project(racerbot_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# `ros2 run racerbot_launch map_course`: the one-command mapping run.
install(PROGRAMS scripts/map_course DESTINATION lib/${PROJECT_NAME})
# systemd/ (the dashboard's boot unit + install.sh) is used from the source
# tree, like the submodule's car/systemd/: nothing to install.

if(BUILD_TESTING)
  # Config checks that need no ROS at all -- runnable directly too:
  #   python3 -m pytest src/racerbot_launch/test/ -v
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_web_dashboard_rb2_config
    test/test_web_dashboard_rb2_config.py)
  ament_add_pytest_test(test_car2_hardware_config
    test/test_car2_hardware_config.py)
  ament_add_pytest_test(test_steering_limits_fit_servo
    test/test_steering_limits_fit_servo.py)
  ament_add_pytest_test(test_map_course
    test/test_map_course.py)
  # The dashboard's boot unit; its last test needs ROS sourced (skips without).
  ament_add_pytest_test(test_dashboard_service
    test/test_dashboard_service.py)
  # Needs ROS sourced (loads the real launch file with launch's LaunchContext).
  ament_add_pytest_test(test_map_course_imu
    test/test_map_course_imu.py)
  ament_add_pytest_test(test_map_course_imu_hardware
    test/test_map_course_imu_hardware.py)
  ament_add_pytest_test(test_imu_flags_both_launches
    test/test_imu_flags_both_launches.py)
endif()

ament_package()
