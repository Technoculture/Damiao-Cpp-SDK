#include "dm_motor_sdk/motor_controller.h"
#include "dm_motor_sdk/protocol_utils.h"
#include <cstring> // For memcpy and memset
#include <chrono>
#include <thread>

namespace dm_motor_sdk {

MotorController::MotorController(SerialDevice* serial_device) {
    serial_device_ = serial_device;
    num_motors_ = 0;
    data_save_len_ = 0;

    // Initialize the send data frame template
    send_data_frame_ = {0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00};

    for (int i = 0; i < MAX_MOTORS; ++i) {
        motors_[i] = nullptr;
    }
}

ControllerErrorCode MotorController::add_motor(Motor* motor) {
    if (num_motors_ >= MAX_MOTORS) {
        return ControllerErrorCode::MOTOR_LIST_FULL;
    }
    motors_[num_motors_] = motor;
    num_motors_++;
    return ControllerErrorCode::SUCCESS;
}

Motor* MotorController::find_motor_by_slave_id(uint16_t slave_id) {
    for (int i = 0; i < num_motors_; ++i) {
        if (motors_[i] != nullptr && motors_[i]->get_slave_id() == slave_id) {
            return motors_[i];
        }
    }
    return nullptr;
}

Motor* MotorController::find_motor_by_master_id(uint16_t master_id) {
    for (int i = 0; i < num_motors_; ++i) {
        if (motors_[i] != nullptr && motors_[i]->get_master_id() == master_id) {
            return motors_[i];
        }
    }
    return nullptr;
}


ControllerErrorCode MotorController::control_mit(Motor* motor, float kp, float kd, float q, float dq, float tau) {
    if (find_motor_by_slave_id(motor->get_slave_id()) == nullptr) {
        return ControllerErrorCode::MOTOR_NOT_FOUND;
    }

    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);

    const MotorLimits& limits = LIMIT_PARAM[static_cast<size_t>(motor->get_motor_type())];
    uint16_t q_uint = float_to_uint(q, -limits.p_max, limits.p_max, 16);
    uint16_t dq_uint = float_to_uint(dq, -limits.v_max, limits.v_max, 12);
    uint16_t tau_uint = float_to_uint(tau, -limits.t_max, limits.t_max, 12);

    std::array<uint8_t, 8> data_buf;
    data_buf[0] = static_cast<uint8_t>((q_uint >> 8) & 0xff);
    data_buf[1] = static_cast<uint8_t>(q_uint & 0xff);
    data_buf[2] = static_cast<uint8_t>(dq_uint >> 4);
    data_buf[3] = static_cast<uint8_t>(((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf));
    data_buf[4] = static_cast<uint8_t>(kp_uint & 0xff);
    data_buf[5] = static_cast<uint8_t>(kd_uint >> 4);
    data_buf[6] = static_cast<uint8_t>(((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf));
    data_buf[7] = static_cast<uint8_t>(tau_uint & 0xff);

    send_data(motor->get_slave_id(), data_buf);
    recv();
    return ControllerErrorCode::SUCCESS;
}

ControllerErrorCode MotorController::control_pos_vel(Motor* motor, float p_desired, float v_desired) {
    if (find_motor_by_slave_id(motor->get_slave_id()) == nullptr) {
        return ControllerErrorCode::MOTOR_NOT_FOUND;
    }
    uint16_t motor_id = 0x100 + motor->get_slave_id();
    std::array<uint8_t, 8> data_buf;

    std::array<uint8_t, 4> p_bytes = float_to_uint8s(p_desired);
    std::array<uint8_t, 4> v_bytes = float_to_uint8s(v_desired);

    memcpy(&data_buf[0], p_bytes.data(), 4);
    memcpy(&data_buf[4], v_bytes.data(), 4);

    send_data(motor_id, data_buf);
    recv();
    return ControllerErrorCode::SUCCESS;
}

ControllerErrorCode MotorController::control_vel(Motor* motor, float v_desired) {
    if (find_motor_by_slave_id(motor->get_slave_id()) == nullptr) {
        return ControllerErrorCode::MOTOR_NOT_FOUND;
    }
    uint16_t motor_id = 0x200 + motor->get_slave_id();
    std::array<uint8_t, 8> data_buf;
    std::array<uint8_t, 4> v_bytes = float_to_uint8s(v_desired);
    memcpy(&data_buf[0], v_bytes.data(), 4);
    memset(&data_buf[4], 0, 4);

    send_data(motor_id, data_buf);
    recv();
    return ControllerErrorCode::SUCCESS;
}

ControllerErrorCode MotorController::control_pos_force(Motor* motor, float pos_des, float vel_des, float i_des) {
     if (find_motor_by_slave_id(motor->get_slave_id()) == nullptr) {
        return ControllerErrorCode::MOTOR_NOT_FOUND;
    }
    uint16_t motor_id = 0x300 + motor->get_slave_id();
    std::array<uint8_t, 8> data_buf;

    std::array<uint8_t, 4> pos_bytes = float_to_uint8s(pos_des);
    memcpy(&data_buf[0], pos_bytes.data(), 4);

    uint16_t vel_uint = static_cast<uint16_t>(vel_des);
    uint16_t i_uint = static_cast<uint16_t>(i_des);

    data_buf[4] = static_cast<uint8_t>(vel_uint & 0xff);
    data_buf[5] = static_cast<uint8_t>(vel_uint >> 8);
    data_buf[6] = static_cast<uint8_t>(i_uint & 0xff);
    data_buf[7] = static_cast<uint8_t>(i_uint >> 8);

    send_data(motor_id, data_buf);
    recv();
    return ControllerErrorCode::SUCCESS;
}

void MotorController::control_cmd(Motor* motor, uint8_t cmd) {
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    send_data(motor->get_slave_id(), data_buf);
}

ControllerErrorCode MotorController::enable_motor(Motor* motor) {
    control_cmd(motor, 0xFC);
    // Note: The sleep calls are direct translations from the original Python code's
    // `sleep()` calls. For a production environment, a more robust, non-blocking
    // asynchronous approach to handling command responses would be recommended.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    recv();
    return ControllerErrorCode::SUCCESS;
}

ControllerErrorCode MotorController::disable_motor(Motor* motor) {
    control_cmd(motor, 0xFD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    recv();
    return ControllerErrorCode::SUCCESS;
}

ControllerErrorCode MotorController::set_zero_position(Motor* motor) {
    control_cmd(motor, 0xFE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    recv();
    return ControllerErrorCode::SUCCESS;
}

void MotorController::send_data(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
    send_data_frame_[13] = static_cast<uint8_t>(motor_id & 0xff);
    send_data_frame_[14] = static_cast<uint8_t>((motor_id >> 8) & 0xff);
    memcpy(&send_data_frame_[21], data.data(), 8);
    serial_device_->write_data(send_data_frame_.data(), static_cast<int>(send_data_frame_.size()));
}

void MotorController::recv() {
    std::array<uint8_t, 512> recv_buffer;
    int bytes_read = serial_device_->read_data(recv_buffer.data(), static_cast<int>(recv_buffer.size()));

    if (bytes_read > 0) {
        std::array<uint8_t, 1024> combined_buffer;
        memcpy(combined_buffer.data(), data_save_buffer_.data(), static_cast<size_t>(data_save_len_));
        memcpy(combined_buffer.data() + data_save_len_, recv_buffer.data(), static_cast<size_t>(bytes_read));
        extract_packets(combined_buffer.data(), data_save_len_ + bytes_read);
    }
}

void MotorController::recv_set_param_data() {
    // This is a simplified version. For robustness, a more complex state machine
    // or timeout mechanism would be needed.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    recv();
}


void MotorController::extract_packets(const uint8_t* data, int length) {
    const uint8_t header = 0xAA;
    const uint8_t tail = 0x55;
    const int frame_length = 16;
    int i = 0;
    int remainder_pos = 0;

    while (i <= length - frame_length) {
        if (data[i] == header && data[i + frame_length - 1] == tail) {
            uint32_t can_id = (data[i+6] << 24) | (data[i+5] << 16) | (data[i+4] << 8) | data[i+3];
            uint8_t cmd = data[i+1];

            std::array<uint8_t, 8> frame_data;
            memcpy(frame_data.data(), &data[i+7], 8);

            process_packet(frame_data, can_id, cmd);

            i += frame_length;
            remainder_pos = i;
        } else {
            i += 1;
        }
    }

    data_save_len_ = length - remainder_pos;
    if (data_save_len_ > 0) {
        memcpy(data_save_buffer_.data(), &data[remainder_pos], static_cast<size_t>(data_save_len_));
    }
}


void MotorController::process_packet(const std::array<uint8_t, 8>& data, uint32_t can_id, uint8_t cmd) {
    if (cmd == 0x11) { // Motor status feedback
         if (can_id != 0x00) {
            Motor* motor = find_motor_by_slave_id(static_cast<uint16_t>(can_id));
            if (motor) {
                uint16_t q_uint = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);
                uint16_t dq_uint = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4));
                uint16_t tau_uint = static_cast<uint16_t>((static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5]);

                const MotorLimits& limits = LIMIT_PARAM[static_cast<size_t>(motor->get_motor_type())];
                float recv_q = uint_to_float(q_uint, -limits.p_max, limits.p_max, 16);
                float recv_dq = uint_to_float(dq_uint, -limits.v_max, limits.v_max, 12);
                float recv_tau = uint_to_float(tau_uint, -limits.t_max, limits.t_max, 12);
                motor->recv_data(recv_q, recv_dq, recv_tau);
            }
        } else {
            uint16_t master_id = static_cast<uint16_t>(data[0] & 0x0f);
            Motor* motor = find_motor_by_master_id(master_id);
            if (motor) {
                uint16_t q_uint = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);
                uint16_t dq_uint = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4));
                uint16_t tau_uint = static_cast<uint16_t>((static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5]);

                const MotorLimits& limits = LIMIT_PARAM[static_cast<size_t>(motor->get_motor_type())];
                float recv_q = uint_to_float(q_uint, -limits.p_max, limits.p_max, 16);
                float recv_dq = uint_to_float(dq_uint, -limits.v_max, limits.v_max, 12);
                float recv_tau = uint_to_float(tau_uint, -limits.t_max, limits.t_max, 12);
                motor->recv_data(recv_q, recv_dq, recv_tau);
            }
        }
    } else { // Parameter setting feedback
        process_set_param_packet(data, can_id, cmd);
    }
}


void MotorController::process_set_param_packet(const std::array<uint8_t, 8>& data, uint32_t can_id, uint8_t cmd) {
    if (cmd == 0x11 && (data[2] == 0x33 || data[2] == 0x55)) {
        uint16_t master_id = static_cast<uint16_t>(can_id);
        uint16_t slave_id = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);

        Motor* motor = find_motor_by_master_id(master_id);
        if (!motor) {
            motor = find_motor_by_slave_id(slave_id);
        }
        if (!motor) return;

        DMVariable rid = static_cast<DMVariable>(data[3]);
        if (is_in_uint32_ranges(rid)) {
            uint32_t num = uint8s_to_uint32(data[4], data[5], data[6], data[7]);
            motor->set_param(rid, static_cast<float>(num));
        } else {
            float num = uint8s_to_float(data[4], data[5], data[6], data[7]);
            motor->set_param(rid, num);
        }
    }
}

ControllerErrorCode MotorController::refresh_motor_status(Motor* motor) {
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0xCC, 0, 0, 0, 0, 0};
    send_data(0x7FF, data_buf);
    recv();
    return ControllerErrorCode::SUCCESS;
}


void MotorController::write_motor_param(Motor* motor, DMVariable rid, float data) {
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0x55, static_cast<uint8_t>(rid), 0, 0, 0, 0};

    if (is_in_uint32_ranges(rid)) {
        std::array<uint8_t, 4> int_bytes = data_to_uint8s(static_cast<uint32_t>(data));
        memcpy(&data_buf[4], int_bytes.data(), 4);
    } else {
        std::array<uint8_t, 4> float_bytes = float_to_uint8s(data);
        memcpy(&data_buf[4], float_bytes.data(), 4);
    }
    send_data(0x7FF, data_buf);
}


void MotorController::read_rid_param(Motor* motor, DMVariable rid) {
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0x33, static_cast<uint8_t>(rid), 0, 0, 0, 0};
    send_data(0x7FF, data_buf);
}


ControllerErrorCode MotorController::switch_control_mode(Motor* motor, ControlType mode) {
    const int max_retries = 20;
    const int retry_interval_ms = 100;
    DMVariable rid = DMVariable::CTRL_MODE;

    write_motor_param(motor, rid, static_cast<float>(static_cast<int>(mode)));

    for (int i = 0; i < max_retries; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
        recv_set_param_data();
        if (motor->has_param(rid)) {
            if (static_cast<int>(motor->get_param(rid)) == static_cast<int>(mode)) {
                return ControllerErrorCode::SUCCESS;
            } else {
                return ControllerErrorCode::WRITE_ERROR;
            }
        }
    }
    return ControllerErrorCode::TIMEOUT;
}

ControllerErrorCode MotorController::save_motor_param(Motor* motor) {
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0xAA, 0, 0, 0, 0, 0};

    disable_motor(motor);
    send_data(0x7FF, data_buf);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return ControllerErrorCode::SUCCESS;
}

float MotorController::read_motor_param(Motor* motor, DMVariable rid) {
    const int max_retries = 20;
    const int retry_interval_ms = 50;

    read_rid_param(motor, rid);
    for (int i = 0; i < max_retries; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
        recv_set_param_data();
        if (motor->has_param(rid)) {
            return motor->get_param(rid);
        }
    }
    return 0.0f; // Return default value on timeout
}

bool MotorController::change_motor_param(Motor* motor, DMVariable rid, float data) {
    const int max_retries = 20;
    const int retry_interval_ms = 50;

    write_motor_param(motor, rid, data);
    for (int i = 0; i < max_retries; ++i) {
        recv_set_param_data();
        if (motor->has_param(rid)) {
            // Using a small epsilon for float comparison
            if (std::abs(motor->get_param(rid) - data) < 0.001f) {
                return true;
            } else {
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
    }
    return false;
}

} // namespace dm_motor_sdk