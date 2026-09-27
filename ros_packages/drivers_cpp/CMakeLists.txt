cmake_minimum_required(VERSION 3.8)
project(drivers_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# add_compile_options(-O3)
add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoboat_msgs REQUIRED)
find_package(Threads REQUIRED)

find_package(serial_driver REQUIRED)
find_library(UDEV_LIB udev)



include_directories(src)
include_directories(src/rc)
include_directories(src/motor_controller)



add_library(vesc_protocol SHARED
  src/motor_controller/vesc_protocol/buffer.cpp
  src/motor_controller/vesc_protocol/crc.cpp
  src/motor_controller/vesc_protocol/vesc_protocol.cpp
)

add_executable(vesc_node src/motor_controller/vesc_node.cpp)
target_link_libraries(vesc_node vesc_protocol ${UDEV_LIB})
ament_target_dependencies(vesc_node rclcpp std_msgs autoboat_msgs serial_driver)

install(TARGETS vesc_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS vesc_node
  DESTINATION lib/${PROJECT_NAME})






set(XCRSF_SRC_LIST 
    src/rc/xcrsf/crossfire.cpp 
    src/rc/xcrsf/serial.cpp 
    src/rc/xcrsf/crc.cpp 
    src/rc/xcrsf/utils.cpp 
    src/rc/xcrsf/handler.cpp
)

add_library(xcrsf SHARED ${XCRSF_SRC_LIST})
target_link_libraries(xcrsf PRIVATE ${CMAKE_THREAD_LIBS_INIT} atomic)



add_executable(rc_node src/rc/rc_node.cpp)
ament_target_dependencies(rc_node rclcpp std_msgs autoboat_msgs)
target_link_libraries(rc_node xcrsf ${CMAKE_THREAD_LIBS_INIT} ${UDEV_LIB})

# Make it so that the xcrsf library is packaged in with the final apt package
install(TARGETS xcrsf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS rc_node DESTINATION lib/${PROJECT_NAME})



add_executable(wind_sensor_node src/wind_sensor/wind_sensor_node.cpp)
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
