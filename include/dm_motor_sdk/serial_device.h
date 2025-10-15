#ifndef DM_MOTOR_SDK_SERIAL_DEVICE_H
#define DM_MOTOR_SDK_SERIAL_DEVICE_H

#include <string>
#include <cstdint>

namespace dm_motor_sdk {

// Simple error codes for serial operations
enum class SerialErrorCode {
    SUCCESS = 0,
    OPEN_FAILED = -1,
    CONFIG_FAILED = -2,
    READ_FAILED = -3,
    WRITE_FAILED = -4,
    CLOSE_FAILED = -5
};

class SerialDevice {
public:
    SerialDevice(const char* port_name, int baud_rate);
    ~SerialDevice();

    SerialErrorCode open_device();
    void close_device();
    bool is_open() const;

    // Read all available bytes from the serial port buffer
    int read_data(uint8_t* buffer, int buffer_size);

    // Write bytes to the serial port
    int write_data(const uint8_t* data, int length);

private:
    const char* port_name_;
    int baud_rate_;
    bool is_open_;

#ifdef _WIN32
    void* handle_; // Using void* to avoid including <windows.h> in the header
#else
    int file_descriptor_;
#endif
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_SERIAL_DEVICE_H