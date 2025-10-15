#ifndef DM_MOTOR_SDK_MOTOR_CONTROLLER_H
#define DM_MOTOR_SDK_MOTOR_CONTROLLER_H

#include "serial_device.h"
#include "motor.h"
#include <array>

namespace dm_motor_sdk {

const int MAX_MOTORS = 16; // Arbitrary limit on the number of motors

// Simple error codes for controller operations
enum class ControllerErrorCode {
    SUCCESS = 0,
    MOTOR_NOT_FOUND = -1,
    MOTOR_LIST_FULL = -2,
    WRITE_ERROR = -3,
    READ_ERROR = -4,
    TIMEOUT = -5
};

class MotorController {
public:
    MotorController(SerialDevice* serial_device);

    ControllerErrorCode add_motor(Motor* motor);

    ControllerErrorCode control_mit(Motor* motor, float kp, float kd, float q, float dq, float tau);
    ControllerErrorCode control_pos_vel(Motor* motor, float p_desired, float v_desired);
    ControllerErrorCode control_vel(Motor* motor, float v_desired);
    ControllerErrorCode control_pos_force(Motor* motor, float pos_des, float vel_des, float i_des);

    ControllerErrorCode enable_motor(Motor* motor);
    ControllerErrorCode disable_motor(Motor* motor);
    ControllerErrorCode set_zero_position(Motor* motor);
    ControllerErrorCode refresh_motor_status(Motor* motor);

    ControllerErrorCode switch_control_mode(Motor* motor, ControlType mode);
    ControllerErrorCode save_motor_param(Motor* motor);

    float read_motor_param(Motor* motor, DMVariable rid);
    bool change_motor_param(Motor* motor, DMVariable rid, float data);

private:
    void send_data(uint16_t motor_id, const std::array<uint8_t, 8>& data);
    void recv();
    void recv_set_param_data();
    void process_packet(const std::array<uint8_t, 8>& data, uint32_t can_id, uint8_t cmd);
    void process_set_param_packet(const std::array<uint8_t, 8>& data, uint32_t can_id, uint8_t cmd);
    void extract_packets(const uint8_t* data, int length);

    void control_cmd(Motor* motor, uint8_t cmd);
    void write_motor_param(Motor* motor, DMVariable rid, float data);
    void read_rid_param(Motor* motor, DMVariable rid);

    Motor* find_motor_by_master_id(uint16_t master_id);
    Motor* find_motor_by_slave_id(uint16_t slave_id);


    SerialDevice* serial_device_;
    Motor* motors_[MAX_MOTORS];
    int num_motors_;

    std::array<uint8_t, 30> send_data_frame_;
    std::array<uint8_t, 256> data_save_buffer_;
    int data_save_len_;
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_MOTOR_CONTROLLER_H