cmake_minimum_required(VERSION 3.8)
project(sensors_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wsign-compare)
  add_compile_options(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoboat_msgs REQUIRED)
find_package(Threads REQUIRED)

find_package(serial_driver REQUIRED)
find_library(UDEV_LIB udev)



include_directories(include include/sensors_cpp)
include_directories(include include/xcrsf)

set(XCRSF_SRC_LIST 
    src/xcrsf/crossfire.cpp 
    src/xcrsf/serial.cpp 
    src/xcrsf/crc.cpp 
    src/xcrsf/utils.cpp 
    src/xcrsf/handler.cpp
)


add_library(xcrsf SHARED ${XCRSF_SRC_LIST})
target_link_libraries(xcrsf PRIVATE ${CMAKE_THREAD_LIBS_INIT} atomic)


add_executable(rc_node src/rc_node.cpp)
ament_target_dependencies(rc_node rclcpp std_msgs autoboat_msgs)
target_link_libraries(rc_node xcrsf ${CMAKE_THREAD_LIBS_INIT} ${UDEV_LIB})
install(TARGETS rc_node DESTINATION lib/${PROJECT_NAME})


add_executable(wind_sensor_node src/wind_sensor_node.cpp)
ament_target_dependencies(wind_sensor_node rclcpp std_msgs geometry_msgs serial_driver)
target_link_libraries(wind_sensor_node ${UDEV_LIB})
install(TARGETS wind_sensor_node DESTINATION lib/${PROJECT_NAME})




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
