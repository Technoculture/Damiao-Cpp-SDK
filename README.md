# DaMiao Motor C++ SDK

This repository contains the C++ SDK for controlling DaMiao series motors. It provides a simple and efficient way to interface with the motors using a serial connection.

## Prerequisites

- A C++ compiler that supports C++23 (e.g., GCC, Clang)
- CMake (version 3.16 or higher)

## Building the Project

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/your-repository.git
    cd your-repository
    ```

2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

3.  **Configure the project with CMake:**
    ```bash
    cmake ..
    ```

4.  **Compile the project:**
    ```bash
    make
    ```
    This will build the `dm_motor_sdk` static library and the `motor_test` example executable.

## Running the Example

The `motor_test` executable demonstrates how to use the library to control a motor. You can run it from the `build` directory:

```bash
./motor_test
```

**Note:** You may need to modify the serial port device name in `examples/motor_test.cpp` to match your system (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).

## Project Structure

- `include/`: Header files for the SDK.
- `src/`: Source files for the SDK library.
- `examples/`: Example code demonstrating how to use the library.
- `CMakeLists.txt`: The main CMake build script.

## Usage

To use the library in your own project, you can link against the `dm_motor_sdk` library. Refer to `examples/motor_test.cpp` for a basic usage pattern.