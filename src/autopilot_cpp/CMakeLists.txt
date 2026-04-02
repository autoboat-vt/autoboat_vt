cmake_minimum_required(VERSION 3.16)
project(autopilot_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for IDE support (clangd, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use ccache if available (huge incremental build speedup)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
endif()

# Use mold linker if available (faster linking)
find_program(MOLD_PROGRAM mold)
if(MOLD_PROGRAM)
  add_link_options(-fuse-ld=mold)
  message(STATUS "Using mold linker: ${MOLD_PROGRAM}")
endif()

add_compile_options(-Wall -Wextra -Wpedantic)



# Find installed dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(autoboat_msgs REQUIRED)
find_package(cpr REQUIRED)
find_package(yaml-cpp REQUIRED)



# Shared autopilot static library (compiled once, linked by multiple targets)
add_library(autopilot_lib STATIC
  autopilot_library/geographic_function_library.cpp
  autopilot_library/autopilot_utils.cpp
  autopilot_library/discrete_pid.cpp
  autopilot_library/position.cpp
  autopilot_library/motorboat_autopilot.cpp
  autopilot_library/sailboat_autopilot.cpp
)
target_include_directories(autopilot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}
)
ament_target_dependencies(autopilot_lib rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(autopilot_lib nlohmann_json::nlohmann_json yaml-cpp)

# Precompile heavy headers once on the library target
target_precompile_headers(autopilot_lib PRIVATE
  <rclcpp/rclcpp.hpp>
  <nlohmann/json.hpp>
  <yaml-cpp/yaml.h>
)



# create motorboat autopilot node executable
add_executable(motorboat_autopilot src/motorboat_autopilot_node.cpp)
target_include_directories(motorboat_autopilot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(motorboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(motorboat_autopilot autopilot_lib nlohmann_json::nlohmann_json yaml-cpp)
target_precompile_headers(motorboat_autopilot REUSE_FROM autopilot_lib)
install(TARGETS motorboat_autopilot DESTINATION lib/${PROJECT_NAME})



# create sailboat autopilot node executable
add_executable(sailboat_autopilot src/sailboat_autopilot_node.cpp)
target_include_directories(sailboat_autopilot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(sailboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(sailboat_autopilot autopilot_lib nlohmann_json::nlohmann_json yaml-cpp)
target_precompile_headers(sailboat_autopilot REUSE_FROM autopilot_lib)
install(TARGETS sailboat_autopilot DESTINATION lib/${PROJECT_NAME})



# create telemetry node executable
add_executable(telemetry src/telemetry_node.cpp src/telemetry_payloads.cpp)
target_include_directories(telemetry PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(telemetry rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(telemetry nlohmann_json::nlohmann_json cpr::cpr OpenSSL::SSL OpenSSL::Crypto yaml-cpp)
target_precompile_headers(telemetry REUSE_FROM autopilot_lib)
install(TARGETS telemetry DESTINATION lib/${PROJECT_NAME})



# add the config files to the install directory
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})



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
