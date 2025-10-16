#ifndef DM_MOTOR_SDK_ERROR_CODES_H
#define DM_MOTOR_SDK_ERROR_CODES_H

#include <cstdint>

namespace dm_motor_sdk {

enum class ErrorCode : int32_t {
    SUCCESS = 0,

    // General errors
    ERROR_UNKNOWN = 1,
    ERROR_NOT_INITIALIZED = 2,
    ERROR_INVALID_ARGUMENT = 3,
    ERROR_OUT_OF_RANGE = 4,
    MOTOR_NOT_FOUND = 5,
    MOTOR_LIST_FULL = 6,

    // Hardware errors
    ERROR_HARDWARE_FAILURE = 100,
    ERROR_COMMUNICATION_TIMEOUT = 101,
    ERROR_SENSOR_DISCONNECTED = 102,
    OPEN_FAILED = 103,
    CONFIG_FAILED = 104,
    READ_FAILED = 105,
    WRITE_FAILED = 106,
    CLOSE_FAILED = 107,

    // Control errors
    ERROR_CONTROL_UNSTABLE = 200,
    ERROR_TRAJECTORY_INVALID = 201,
    ERROR_JOINT_LIMIT_EXCEEDED = 202,
};

inline const char* error_to_string(ErrorCode code) {
    switch(code) {
        case ErrorCode::SUCCESS: return "Success";
        case ErrorCode::ERROR_UNKNOWN: return "Unknown error";
        case ErrorCode::ERROR_NOT_INITIALIZED: return "Not initialized";
        case ErrorCode::ERROR_INVALID_ARGUMENT: return "Invalid argument";
        case ErrorCode::ERROR_OUT_OF_RANGE: return "Out of range";
        case ErrorCode::MOTOR_NOT_FOUND: return "Motor not found";
        case ErrorCode::MOTOR_LIST_FULL: return "Motor list full";
        case ErrorCode::ERROR_HARDWARE_FAILURE: return "Hardware failure";
        case ErrorCode::ERROR_COMMUNICATION_TIMEOUT: return "Communication timeout";
        case ErrorCode::ERROR_SENSOR_DISCONNECTED: return "Sensor disconnected";
        case ErrorCode::OPEN_FAILED: return "Failed to open device";
        case ErrorCode::CONFIG_FAILED: return "Failed to configure device";
        case ErrorCode::READ_FAILED: return "Read operation failed";
        case ErrorCode::WRITE_FAILED: return "Write operation failed";
        case ErrorCode::CLOSE_FAILED: return "Failed to close device";
        case ErrorCode::ERROR_CONTROL_UNSTABLE: return "Control unstable";
        case ErrorCode::ERROR_TRAJECTORY_INVALID: return "Trajectory invalid";
        case ErrorCode::ERROR_JOINT_LIMIT_EXCEEDED: return "Joint limit exceeded";
        default: return "Unknown error";
    }
}

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_ERROR_CODES_H