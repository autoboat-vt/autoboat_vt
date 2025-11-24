cmake_minimum_required(VERSION 2.8)
project(vescuart-slow-rpm-increase)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Vescuart REQUIRED)

set(EXAMPLE_SOURCES
    src/Main.cpp
)

include_directories(include
    ${VESCUART_INCLUDE_DIRS})

add_executable(vescuart-example ${EXAMPLE_SOURCES})
target_link_libraries(vescuart-example
    ${VESCUART_LIBRARIES})
