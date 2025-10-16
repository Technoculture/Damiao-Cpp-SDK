#ifndef DM_MOTOR_SDK_SERIAL_DEVICE_INTERFACE_H
#define DM_MOTOR_SDK_SERIAL_DEVICE_INTERFACE_H

#include "error_codes.h"
#include <cstdint>

namespace dm_motor_sdk {

class SerialDeviceInterface {
public:
    virtual ~SerialDeviceInterface() = default;
    virtual ErrorCode open_device() = 0;
    virtual void close_device() = 0;
    virtual bool is_open() const = 0;
    virtual int read_data(uint8_t* buffer, int buffer_size) = 0;
    virtual int write_data(const uint8_t* data, int length) = 0;
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_SERIAL_DEVICE_INTERFACE_H