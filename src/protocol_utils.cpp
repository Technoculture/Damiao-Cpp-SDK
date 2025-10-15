#include "dm_motor_sdk/protocol_utils.h"
#include <cstring> // For memcpy

namespace dm_motor_sdk {

float limit_min_max(float x, float min, float max) {
    if (x < min) {
        return min;
    }
    if (x > max) {
        return max;
    }
    return x;
}

uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    x = limit_min_max(x, x_min, x_max);
    float span = x_max - x_min;
    float data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

float uint_to_float(uint16_t x, float min, float max, int bits) {
    float span = max - min;
    float data_norm = static_cast<float>(x) / ((1 << bits) - 1);
    return data_norm * span + min;
}

std::array<uint8_t, 4> float_to_uint8s(float value) {
    std::array<uint8_t, 4> bytes;
    memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

std::array<uint8_t, 4> data_to_uint8s(uint32_t value) {
    std::array<uint8_t, 4> bytes;
    memcpy(bytes.data(), &value, sizeof(uint32_t));
    return bytes;
}

bool is_in_uint32_ranges(DMVariable number) {
    int num = static_cast<int>(number);
    if ((7 <= num && num <= 10) || (13 <= num && num <= 16) || (35 <= num && num <= 36)) {
        return true;
    }
    return false;
}

uint32_t uint8s_to_uint32(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
    uint32_t value;
    uint8_t bytes[] = {b1, b2, b3, b4};
    memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}

float uint8s_to_float(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
    float value;
    uint8_t bytes[] = {b1, b2, b3, b4};
    memcpy(&value, bytes, sizeof(float));
    return value;
}

} // namespace dm_motor_sdk