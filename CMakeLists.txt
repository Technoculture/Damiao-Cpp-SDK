cmake_minimum_required(VERSION 3.16)
project(DamiaoMotorSDK CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Style guide compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti -Wno-unused-parameter -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wconversion -Wsign-conversion -Wnull-dereference")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wformat=2")


# Sanitizer flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -fno-omit-frame-pointer")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,undefined")

# Header files
include_directories(include)

# Create the library
add_library(dm_motor_sdk src/serial_device.cpp
                           src/motor.cpp
                           src/motor_controller.cpp
                           src/protocol_utils.cpp)

# Create the example executable
add_executable(motor_test examples/motor_test.cpp)

# Link the library to the example
target_link_libraries(motor_test dm_motor_sdk)