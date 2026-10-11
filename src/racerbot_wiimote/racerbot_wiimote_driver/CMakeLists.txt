cmake_minimum_required(VERSION 3.20)
project(racerbot_wiimote_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(racerbot_wiimote_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
# Not REQUIRED: without xwiimote the rest of the workspace must still build.
pkg_check_modules(XWIIMOTE libxwiimote)

# Wiimote input -> Joy rules, with no ROS or xwiimote, so they are unit tested.
add_library(wiimote_joy_mapping STATIC src/joy_mapping.cpp)
target_include_directories(wiimote_joy_mapping PUBLIC include/)

if(XWIIMOTE_FOUND)
  add_executable(wiimote_driver_node src/wiimote_driver_node.cpp src/wiimote_device.cpp)
  target_include_directories(wiimote_driver_node PRIVATE ${XWIIMOTE_INCLUDE_DIRS} include/)
  target_link_directories(wiimote_driver_node PRIVATE ${XWIIMOTE_LIBRARY_DIRS})
  # Keyword-form target_link_libraries only. The previous file fell back to
  # ament_target_dependencies() on ament_cmake < 2.7.3, i.e. on Jazzy, and
  # mixing the two forms is a CMake error: the package never built on the car.
  target_link_libraries(wiimote_driver_node PRIVATE
    wiimote_joy_mapping
    ${XWIIMOTE_LIBRARIES}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${racerbot_wiimote_msgs_TARGETS}
  )
  install(TARGETS wiimote_driver_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "libxwiimote not found: wiimote_driver_node is NOT built (everything else is). "
    "To drive with a Wiimote: sudo apt install xwiimote libxwiimote-dev, then rebuild. "
    "See docs/wiimote.md.")
endif()
install(PROGRAMS scripts/wiimote_connect DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joy_mapping test/test_joy_mapping.cpp)
  target_link_libraries(test_joy_mapping wiimote_joy_mapping)
  # Needs no ROS: python3 -m pytest src/racerbot_wiimote/racerbot_wiimote_driver/test/
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wiimote_connect test/test_wiimote_connect.py)
endif()

ament_package()
