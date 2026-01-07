cmake_minimum_required(VERSION 3.8)

option(BUILD_UNIT_TESTS "Build host side unit tests" OFF)

if(BUILD_UNIT_TESTS)
    # Host CLI unit tests using ROS2 and Catch2
    project(pico_microros_tests CXX)
    set(CMAKE_CXX_STANDARD 17)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    add_executable(unit_tests src/unittests.cpp)
    target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ament_target_dependencies(unit_tests rclcpp std_msgs)

    enable_testing()
    add_test(NAME unit_tests COMMAND unit_tests)

    return()  # Skip Pico SDK build when building tests
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_microros C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories($ENV{PICO_MICROROS_SDK_PATH}/libmicroros)

add_executable(pico_microros
    src/main.cpp

    $ENV{PICO_MICROROS_SDK_PATH}/pico_uart_transport.c
)

target_link_libraries(pico_microros
    pico_stdlib
    microros
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_pwm
)

target_include_directories(pico_microros PUBLIC
    $ENV{CMAKE_CURRENT_SOURCE_DIR}/src
    $ENV{PICO_MICROROS_SDK_PATH}
    $ENV{PICO_MICROROS_SDK_PATH}/libmicroros
    $ENV{PICO_MICROROS_SDK_PATH}/libmicroros/include
    $ENV{PICO_SDK_PATH}/src/rp2_common/pico_multicore/include
    $ENV{PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_microros 1)
pico_enable_stdio_uart(pico_microros 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_microros)