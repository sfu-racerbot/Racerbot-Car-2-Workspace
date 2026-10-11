cmake_minimum_required(VERSION 3.20)
project(racerbot_wiimote_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WiimoteBattery.msg"
  "msg/WiimoteButtons.msg"
  "msg/WiimoteLED.msg"
  "msg/WiimoteAccelerometer.msg"
  "msg/WiimoteRumble.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
