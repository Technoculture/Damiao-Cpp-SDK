#ifndef DM_MOTOR_SDK_MOTOR_CONTROLLER_H
#define DM_MOTOR_SDK_MOTOR_CONTROLLER_H

#include "serial_device_interface.h"
#include "motor.h"
#include <array>
#include <memory>

namespace dm_motor_sdk {

const int MAX_MOTORS = 16; // Arbitrary limit on the number of motors

class MotorController {
public:
    MotorController(SerialDeviceInterface* serial_device);

    ErrorCode add_motor(std::unique_ptr<Motor> motor);

    ErrorCode control_mit(uint16_t motor_id, float kp, float kd, float q, float dq, float tau);
    ErrorCode control_pos_vel(uint16_t motor_id, float p_desired, float v_desired);
    ErrorCode control_vel(uint16_t motor_id, float v_desired);
    ErrorCode control_pos_force(uint16_t motor_id, float pos_des, float vel_des, float i_des);

    ErrorCode enable_motor(uint16_t motor_id);
    ErrorCode disable_motor(uint16_t motor_id);
    ErrorCode set_zero_position(uint16_t motor_id);
    ErrorCode refresh_motor_status(uint16_t motor_id);

    ErrorCode switch_control_mode(uint16_t motor_id, ControlType mode);
    ErrorCode save_motor_param(uint16_t motor_id);

    float read_motor_param(uint16_t motor_id, DMVariable rid);
    bool change_motor_param(uint16_t motor_id, DMVariable rid, float data);

private:
    void send_data(uint16_t motor_id, const std::array<uint8_t, 8>& data);
    void recv();
    void recv_set_param_data();
    void process_packet(const std::array<uint8_t, 8>& data, uint32_t can_id, uint8_t cmd);
    void process_set_param_packet(const std::array<uint8_t, 8>& data, uint32_t can_id, uint8_t cmd);
    void extract_packets(const uint8_t* data, int length);

    void control_cmd(uint16_t motor_id, uint8_t cmd);
    void write_motor_param(uint16_t motor_id, DMVariable rid, float data);
    void read_rid_param(uint16_t motor_id, DMVariable rid);

    Motor* find_motor_by_master_id(uint16_t master_id);
    Motor* find_motor_by_slave_id(uint16_t slave_id);
    Motor* get_motor(uint16_t slave_id);


    SerialDeviceInterface* serial_device;
    std::array<std::unique_ptr<Motor>, MAX_MOTORS> motors;
    int num_motors;

    std::array<uint8_t, 30> send_data_frame;
    std::array<uint8_t, 256> data_save_buffer;
    int data_save_len;
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_MOTOR_CONTROLLER_H