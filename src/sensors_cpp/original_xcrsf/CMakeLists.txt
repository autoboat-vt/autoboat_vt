cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(xcrsf)

find_package(Threads REQUIRED)

# Set C++ standards
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SRC_LIST src/crossfire.cpp src/serial.cpp src/crc.cpp src/utils.cpp src/handler.cpp)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Build shared library
add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_THREAD_LIBS_INIT} atomic)

# Build demo project
add_executable(${PROJECT_NAME}_demo examples/main.cpp)
target_link_libraries(${PROJECT_NAME}_demo PRIVATE ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# Installation directories and rules
set(INSTALL_LIB_DIR lib)
set(INSTALL_INCLUDE_DIR include)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${INSTALL_LIB_DIR} ARCHIVE DESTINATION ${INSTALL_LIB_DIR})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION ${INSTALL_INCLUDE_DIR})