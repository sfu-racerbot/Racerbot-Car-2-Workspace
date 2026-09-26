cmake_minimum_required(VERSION 3.8)
project(racerbot_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Config checks that need no ROS at all -- runnable directly too:
  #   python3 -m pytest src/racerbot_launch/test/ -v
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_foxglove_bridge_config
    test/test_foxglove_bridge_config.py)
endif()

ament_package()
