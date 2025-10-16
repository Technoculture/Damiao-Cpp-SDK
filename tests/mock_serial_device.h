#ifndef DM_MOTOR_SDK_MOCK_SERIAL_DEVICE_H
#define DM_MOTOR_SDK_MOCK_SERIAL_DEVICE_H

#include "dm_motor_sdk/serial_device_interface.h"
#include <vector>

class MockSerialDevice : public dm_motor_sdk::SerialDeviceInterface {
public:
    dm_motor_sdk::ErrorCode open_device() override { return dm_motor_sdk::ErrorCode::SUCCESS; }
    void close_device() override {}
    bool is_open() const override { return true; }

    int read_data(uint8_t* buffer, int buffer_size) override {
        int bytes_to_read = std::min(static_cast<int>(read_buffer.size()), buffer_size);
        if (bytes_to_read > 0) {
            memcpy(buffer, read_buffer.data(), static_cast<size_t>(bytes_to_read));
            read_buffer.erase(read_buffer.begin(), read_buffer.begin() + bytes_to_read);
        }
        return bytes_to_read;
    }

    int write_data(const uint8_t* data, int length) override {
        write_buffer.insert(write_buffer.end(), data, data + length);
        return length;
    }

    void set_read_buffer(const std::vector<uint8_t>& data) {
        read_buffer = data;
    }

    std::vector<uint8_t> write_buffer;
    std::vector<uint8_t> read_buffer;
};

#endif // DM_MOTOR_SDK_MOCK_SERIAL_DEVICE_H