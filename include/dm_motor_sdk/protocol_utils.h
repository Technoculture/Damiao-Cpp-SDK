#ifndef DM_MOTOR_SDK_PROTOCOL_UTILS_H
#define DM_MOTOR_SDK_PROTOCOL_UTILS_H

#include <cstdint>
#include <array>
#include "motor_types.h"

namespace dm_motor_sdk {

// Clamp value to a min/max range
float limit_min_max(float x, float min, float max);

// Convert float to unsigned integer
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

// Convert unsigned integer to float
float uint_to_float(uint16_t x, float min, float max, int bits);

// Convert float to 4 bytes
std::array<uint8_t, 4> float_to_uint8s(float value);

// Convert uint32 to 4 bytes
std::array<uint8_t, 4> data_to_uint8s(uint32_t value);

// Check if a register ID corresponds to a uint32 type
bool is_in_uint32_ranges(DMVariable number);

// Convert 4 bytes to uint32
uint32_t uint8s_to_uint32(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

// Convert 4 bytes to float
float uint8s_to_float(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_PROTOCOL_UTILS_H