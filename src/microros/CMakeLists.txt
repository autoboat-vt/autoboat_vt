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
cmake_policy(SET CMP0079 NEW)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#for RTOS_main.cpp
set(FREERTOS_PORT GCC_RP2040 CACHE STRING "hehe") #which CPU port to use
set(FREERTOS_HEAP 4 CACHE STRING "") #opens memory
add_library(freertos_config INTERFACE) #placeholder
target_include_directories(freertos_config INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/freertos) #just adds directories that links config file to the kernel
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/freertos freertos) #freeRTOS directory, makes kernel
target_link_libraries(freertos_kernel PRIVATE pico_stdlib) #shares pico library with kernel
target_link_libraries(freertos_kernel_port PRIVATE pico_stdlib) #gives port headers too


link_directories($ENV{PICO_MICROROS_SDK_PATH}/libmicroros)

add_executable(pico_microros
    src/main.cpp
    src/microros.cpp 
    src/spi_device.cpp
    src/amt22_encoder.cpp
    src/i2c_device.cpp
    src/cmps14_compass.cpp
    src/systems.cpp
    src/HAL.cpp
    $ENV{PICO_MICROROS_SDK_PATH}/pico_uart_transport.c
)

add_executable(pico-RTOS
    src/rtos_main.cpp
)

target_include_directories(pico-RTOS PUBLIC /home/ws/src/microros/src)

target_link_libraries(pico-RTOS
    freertos_kernel
    freertos_config
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_pwm
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src
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

pico_enable_stdio_usb(pico-RTOS 1)
pico_enable_stdio_uart(pico-RTOS 0)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_microros)
pico_add_extra_outputs(pico-RTOS)
