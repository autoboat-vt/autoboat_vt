cmake_minimum_required(VERSION 3.8)
project(vesc_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoboat_msgs REQUIRED)
find_package(serial_driver REQUIRED)

find_library(UDEV_LIB udev)

include_directories(src)

add_library(vesc_protocol SHARED
  src/vesc_protocol/buffer.cpp
  src/vesc_protocol/crc.cpp
  src/vesc_protocol/vesc_protocol.cpp
)

add_executable(vesc_node src/vesc_node.cpp)

ament_target_dependencies(vesc_node rclcpp std_msgs autoboat_msgs serial_driver)
target_link_libraries(vesc_node vesc_protocol ${UDEV_LIB})

install(TARGETS vesc_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS vesc_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
