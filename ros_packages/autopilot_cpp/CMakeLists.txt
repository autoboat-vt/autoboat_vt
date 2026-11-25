cmake_minimum_required(VERSION 3.16)
project(autopilot_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json to allow the IDE to have proper intellisense
# https://code.visualstudio.com/docs/cpp/configure-intellisense#_compilecommandsjson-file
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

add_compile_options(-Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections)



# Find installed dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(autoboat_msgs REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)



# FetchContent: cpr
include(FetchContent)

set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Use system curl" FORCE)

FetchContent_Declare(
  cpr
  GIT_REPOSITORY https://github.com/libcpr/cpr.git
  GIT_TAG        1.14.2
)

# BEGIN SECTION
# cpr's checkout reads merge.conflictStyle; older Git rejects the zdiff3 value
# if it is present in the user's global config. Override it only for this fetch
# rather than requiring a Git upgrade or changing the user's global config.
# fatal: unknown style 'zdiff3' given for 'merge.conflictstyle'

# GIT_CONFIG_COUNT entries are indexed from zero. Appending preserves any
# overrides supplied by the environment instead of replacing them.
if(DEFINED ENV{GIT_CONFIG_COUNT})
  set(_CPR_PREVIOUS_GIT_CONFIG_COUNT "$ENV{GIT_CONFIG_COUNT}")
  math(EXPR _CPR_GIT_CONFIG_INDEX "${_CPR_PREVIOUS_GIT_CONFIG_COUNT}")
  set(_CPR_GIT_CONFIG_COUNT_WAS_SET TRUE)
else()
  set(_CPR_GIT_CONFIG_INDEX 0)
endif()

# Git requires each indexed override to be supplied as a matching KEY_n/VALUE_n pair.
set(_CPR_GIT_CONFIG_KEY_ENV "GIT_CONFIG_KEY_${_CPR_GIT_CONFIG_INDEX}")
set(_CPR_GIT_CONFIG_VALUE_ENV "GIT_CONFIG_VALUE_${_CPR_GIT_CONFIG_INDEX}")

# The next slot may already have values even though it is not counted; save them
# so this temporary override does not discard unrelated environment state.
if(DEFINED ENV{${_CPR_GIT_CONFIG_KEY_ENV}})
  set(_CPR_PREVIOUS_GIT_CONFIG_KEY "$ENV{${_CPR_GIT_CONFIG_KEY_ENV}}")
  set(_CPR_GIT_CONFIG_KEY_WAS_SET TRUE)
endif()
if(DEFINED ENV{${_CPR_GIT_CONFIG_VALUE_ENV}})
  set(_CPR_PREVIOUS_GIT_CONFIG_VALUE "$ENV{${_CPR_GIT_CONFIG_VALUE_ENV}}")
  set(_CPR_GIT_CONFIG_VALUE_WAS_SET TRUE)
endif()

# Increase the active entry count so Git reads this pair. diff3 is supported by
# older Git and overrides zdiff3 without changing other Git configuration.
math(EXPR _CPR_GIT_CONFIG_COUNT "${_CPR_GIT_CONFIG_INDEX} + 1")
set(ENV{GIT_CONFIG_COUNT} "${_CPR_GIT_CONFIG_COUNT}")
set(ENV{${_CPR_GIT_CONFIG_KEY_ENV}} "merge.conflictStyle")
set(ENV{${_CPR_GIT_CONFIG_VALUE_ENV}} "diff3")

# FetchContent runs the cpr clone/checkout during configuration, and its Git
# subprocesses inherit this override.
FetchContent_MakeAvailable(cpr)

# Restore the indexed variables so later Git subprocesses see the original
# environment rather than this cpr-specific compatibility override.
if(_CPR_GIT_CONFIG_KEY_WAS_SET)
  set(ENV{${_CPR_GIT_CONFIG_KEY_ENV}} "${_CPR_PREVIOUS_GIT_CONFIG_KEY}")
else()
  unset(ENV{${_CPR_GIT_CONFIG_KEY_ENV}})
endif()
if(_CPR_GIT_CONFIG_VALUE_WAS_SET)
  set(ENV{${_CPR_GIT_CONFIG_VALUE_ENV}} "${_CPR_PREVIOUS_GIT_CONFIG_VALUE}")
else()
  unset(ENV{${_CPR_GIT_CONFIG_VALUE_ENV}})
endif()
# Restore the original count so the appended pair is no longer active.
if(_CPR_GIT_CONFIG_COUNT_WAS_SET)
  set(ENV{GIT_CONFIG_COUNT} "${_CPR_PREVIOUS_GIT_CONFIG_COUNT}")
else()
  unset(ENV{GIT_CONFIG_COUNT})
endif()

# At this point the cpr fetch is complete, and the override is no longer needed. The
# environment is restored to its original state so later Git subprocesses are not affected.
# END SECTION

# Shared autopilot static library (compiled once, linked by multiple targets)
add_library(autopilot_lib STATIC
  autopilot_library/geographic_function_library.cpp
  autopilot_library/autopilot_utils.cpp
  autopilot_library/discrete_pid.cpp
  autopilot_library/position.cpp
  autopilot_library/motorboat_autopilot.cpp
  autopilot_library/sailboat_autopilot.cpp
  autopilot_library/telemetry_payloads.cpp
)
target_include_directories(autopilot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}
)
ament_target_dependencies(autopilot_lib rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(autopilot_lib nlohmann_json::nlohmann_json)

# Precompile heavy headers once on the library target
target_precompile_headers(autopilot_lib PRIVATE
  <rclcpp/rclcpp.hpp>
  <nlohmann/json.hpp>
)



# create motorboat autopilot node executable
add_executable(motorboat_autopilot src/motorboat_autopilot_node.cpp)
target_include_directories(motorboat_autopilot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(motorboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(motorboat_autopilot autopilot_lib nlohmann_json::nlohmann_json)
target_precompile_headers(motorboat_autopilot REUSE_FROM autopilot_lib)
install(TARGETS motorboat_autopilot DESTINATION lib/${PROJECT_NAME})



# create sailboat autopilot node executable
add_executable(sailboat_autopilot src/sailboat_autopilot_node.cpp)
target_include_directories(sailboat_autopilot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(sailboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(sailboat_autopilot autopilot_lib nlohmann_json::nlohmann_json)
target_precompile_headers(sailboat_autopilot REUSE_FROM autopilot_lib)
install(TARGETS sailboat_autopilot DESTINATION lib/${PROJECT_NAME})



# create telemetry node executable
add_executable(telemetry src/telemetry_node.cpp)
target_include_directories(telemetry PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR} PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(telemetry rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(telemetry autopilot_lib nlohmann_json::nlohmann_json cpr::cpr OpenSSL::SSL OpenSSL::Crypto ${OpenCV_LIBRARIES})
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
