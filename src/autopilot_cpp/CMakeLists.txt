cmake_minimum_required(VERSION 3.8)
project(autopilot_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



add_compile_options(-O3)
add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)





# Download required libraries
# include(FetchContent)

# FetchContent_Declare(cpr GIT_REPOSITORY https://github.com/libcpr/cpr.git GIT_TAG 1.14.1)
# FetchContent_MakeAvailable(cpr)


# TODO: try to remove this dependency
# FetchContent_Declare(GeographicLib GIT_REPOSITORY https://github.com/geographiclib/geographiclib.git GIT_TAG v2.7)
# FetchContent_MakeAvailable(GeographicLib)



# Find installed dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)   
# find_package(OpenCV REQUIRED)         # TODO try to remove this dependency
# find_package(CURL REQUIRED)           # TODO try to remove this dependency
find_package(nlohmann_json REQUIRED)
find_package(autoboat_msgs REQUIRED)
find_package(cpr REQUIRED)
find_package(yaml-cpp REQUIRED)



# create telemetry node executable
add_executable(telemetry src/telemetry_node.cpp)
ament_target_dependencies(telemetry rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(telemetry nlohmann_json::nlohmann_json cpr::cpr)
install(TARGETS telemetry DESTINATION lib/${PROJECT_NAME})



# create motorboat autopilot node executable
add_executable(motorboat_autopilot src/motorboat_autopilot_node.cpp)
target_include_directories(motorboat_autopilot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(motorboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(motorboat_autopilot nlohmann_json::nlohmann_json yaml-cpp)
install(TARGETS motorboat_autopilot DESTINATION lib/${PROJECT_NAME})


# create sailboat autopilot node executable
add_executable(sailboat_autopilot src/sailboat_autopilot_node.cpp)
target_include_directories(sailboat_autopilot PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(sailboat_autopilot rclcpp std_msgs geometry_msgs sensor_msgs autoboat_msgs)
target_link_libraries(sailboat_autopilot nlohmann_json::nlohmann_json yaml-cpp)
install(TARGETS sailboat_autopilot DESTINATION lib/${PROJECT_NAME})


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
