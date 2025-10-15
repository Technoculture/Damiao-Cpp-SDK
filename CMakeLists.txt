cmake_minimum_required(VERSION 3.16)
project(DamiaoMotorSDK CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Style guide compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fno-exceptions -fno-rtti -Wno-unused-parameter -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wconversion -Wsign-conversion -Wnull-dereference")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wformat=2")
add_definitions(-DSPDLOG_NO_EXCEPTIONS)

# Dependencies
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Catch2 REQUIRED)
find_package(Threads REQUIRED)

# Header files
include_directories(include)

# Source files for the library
set(LIB_SOURCES
    src/serial_device.cpp
    src/motor.cpp
    src/motor_controller.cpp
    src/protocol_utils.cpp
)

# Create the library
add_library(dm_motor_sdk ${LIB_SOURCES})
target_link_libraries(dm_motor_sdk PRIVATE fmt::fmt spdlog::spdlog Threads::Threads)

# Test executable
add_executable(unit_tests tests/test_protocol_utils.cpp tests/test_integration.cpp)
target_link_libraries(unit_tests PRIVATE dm_motor_sdk Catch2::Catch2WithMain)

# Enable testing
enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)