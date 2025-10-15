#ifndef DM_MOTOR_SDK_SERIAL_DEVICE_H
#define DM_MOTOR_SDK_SERIAL_DEVICE_H

#include "serial_device_interface.h"
#include <string>
#include <cstdint>

namespace dm_motor_sdk {

class SerialDevice : public SerialDeviceInterface {
public:
    SerialDevice(const char* port_name, int baud_rate);
    ~SerialDevice() override;

    ErrorCode open_device() override;
    void close_device() override;
    bool is_open() const override;

    // Read all available bytes from the serial port buffer
    int read_data(uint8_t* buffer, int buffer_size) override;

    // Write bytes to the serial port
    int write_data(const uint8_t* data, int length) override;

private:
    const char* port_name;
    int baud_rate;
    bool is_device_open;

#ifdef _WIN32
    void* handle; // Using void* to avoid including <windows.h> in the header
#else
    int file_descriptor;
#endif
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_SERIAL_DEVICE_H