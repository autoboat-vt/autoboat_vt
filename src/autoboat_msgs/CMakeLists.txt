cmake_minimum_required(VERSION 3.8)
project(autoboat_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/VESCTelemetryData.msg"
  "msg/VESCControlData.msg"
  "msg/RCData.msg"
  "msg/WaypointList.msg"
  "msg/ObjectDetectionResult.msg"
  "msg/ObjectDetectionResultsList.msg"
  DEPENDENCIES sensor_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
