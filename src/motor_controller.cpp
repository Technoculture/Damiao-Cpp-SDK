#include "dm_motor_sdk/motor_controller.h"
#include "dm_motor_sdk/protocol_utils.h"
#include <cstring> // For memcpy and memset
#include <chrono>
#include <thread>

namespace dm_motor_sdk {

MotorController::MotorController(SerialDeviceInterface* new_serial_device) {
    this->serial_device = new_serial_device;
    this->num_motors = 0;
    this->data_save_len = 0;

    // Initialize the send data frame template
    this->send_data_frame = {0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00};

    for (auto& motor : motors) {
        motor = nullptr;
    }
}

ErrorCode MotorController::add_motor(std::unique_ptr<Motor> motor) {
    if (static_cast<size_t>(num_motors) >= MAX_MOTORS) {
        return ErrorCode::MOTOR_LIST_FULL;
    }
    motors[static_cast<size_t>(num_motors)] = std::move(motor);
    num_motors++;
    return ErrorCode::SUCCESS;
}

Motor* MotorController::find_motor_by_slave_id(uint16_t slave_id) {
    for (int i = 0; i < num_motors; ++i) {
        if (motors[static_cast<size_t>(i)] != nullptr && motors[static_cast<size_t>(i)]->get_slave_id() == slave_id) {
            return motors[static_cast<size_t>(i)].get();
        }
    }
    return nullptr;
}

Motor* MotorController::get_motor(uint16_t slave_id) {
    return find_motor_by_slave_id(slave_id);
}

Motor* MotorController::find_motor_by_master_id(uint16_t master_id) {
    for (int i = 0; i < num_motors; ++i) {
        if (motors[static_cast<size_t>(i)] != nullptr && motors[static_cast<size_t>(i)]->get_master_id() == master_id) {
            return motors[static_cast<size_t>(i)].get();
        }
    }
    return nullptr;
}


ErrorCode MotorController::control_mit(uint16_t motor_id, float kp, float kd, float q, float dq, float tau) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
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
    return ErrorCode::SUCCESS;
}

ErrorCode MotorController::control_pos_vel(uint16_t motor_id, float p_desired, float v_desired) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
    }
    std::array<uint8_t, 8> data_buf;

    std::array<uint8_t, 4> p_bytes = float_to_uint8s(p_desired);
    std::array<uint8_t, 4> v_bytes = float_to_uint8s(v_desired);

    memcpy(&data_buf[0], p_bytes.data(), 4);
    memcpy(&data_buf[4], v_bytes.data(), 4);

    send_data(0x100 + motor_id, data_buf);
    recv();
    return ErrorCode::SUCCESS;
}

ErrorCode MotorController::control_vel(uint16_t motor_id, float v_desired) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
    }
    uint16_t can_id = 0x200 + motor_id;
    std::array<uint8_t, 8> data_buf;
    std::array<uint8_t, 4> v_bytes = float_to_uint8s(v_desired);
    memcpy(&data_buf[0], v_bytes.data(), 4);
    memset(&data_buf[4], 0, 4);

    send_data(can_id, data_buf);
    recv();
    return ErrorCode::SUCCESS;
}

ErrorCode MotorController::control_pos_force(uint16_t motor_id, float pos_des, float vel_des, float i_des) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
    }
    std::array<uint8_t, 8> data_buf;

    std::array<uint8_t, 4> pos_bytes = float_to_uint8s(pos_des);
    memcpy(&data_buf[0], pos_bytes.data(), 4);

    uint16_t vel_uint = static_cast<uint16_t>(vel_des);
    uint16_t i_uint = static_cast<uint16_t>(i_des);

    data_buf[4] = static_cast<uint8_t>(vel_uint & 0xff);
    data_buf[5] = static_cast<uint8_t>(vel_uint >> 8);
    data_buf[6] = static_cast<uint8_t>(i_uint & 0xff);
    data_buf[7] = static_cast<uint8_t>(i_uint >> 8);

    send_data(0x300 + motor_id, data_buf);
    recv();
    return ErrorCode::SUCCESS;
}

void MotorController::control_cmd(uint16_t motor_id, uint8_t cmd) {
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    send_data(motor_id, data_buf);
}

ErrorCode MotorController::enable_motor(uint16_t motor_id) {
    control_cmd(motor_id, 0xFC);
    // WARNING: This blocking sleep is not suitable for real-time applications.
    // This is a direct translation from the original Python code's `sleep()` calls.
    // For a production environment, a more robust, non-blocking asynchronous
    // approach to handling command responses would be recommended.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    recv();
    return ErrorCode::SUCCESS;
}

ErrorCode MotorController::disable_motor(uint16_t motor_id) {
    control_cmd(motor_id, 0xFD);
    // WARNING: This blocking sleep is not suitable for real-time applications.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    recv();
    return ErrorCode::SUCCESS;
}

ErrorCode MotorController::set_zero_position(uint16_t motor_id) {
    control_cmd(motor_id, 0xFE);
    // WARNING: This blocking sleep is not suitable for real-time applications.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    recv();
    return ErrorCode::SUCCESS;
}

void MotorController::send_data(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
    send_data_frame[13] = static_cast<uint8_t>(motor_id & 0xff);
    send_data_frame[14] = static_cast<uint8_t>((motor_id >> 8) & 0xff);
    memcpy(&send_data_frame[21], data.data(), 8);
    serial_device->write_data(send_data_frame.data(), static_cast<int>(send_data_frame.size()));
}

void MotorController::recv() {
    std::array<uint8_t, 512> recv_buffer;
    int bytes_read = serial_device->read_data(recv_buffer.data(), static_cast<int>(recv_buffer.size()));

    if (bytes_read > 0) {
        std::array<uint8_t, 1024> combined_buffer;
        memcpy(combined_buffer.data(), data_save_buffer.data(), static_cast<size_t>(data_save_len));
        memcpy(combined_buffer.data() + data_save_len, recv_buffer.data(), static_cast<size_t>(bytes_read));
        extract_packets(combined_buffer.data(), data_save_len + bytes_read);
    }
}

void MotorController::recv_set_param_data() {
    // WARNING: This blocking sleep is not suitable for real-time applications.
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

    data_save_len = length - remainder_pos;
    if (data_save_len > 0) {
        memcpy(data_save_buffer.data(), &data[remainder_pos], static_cast<size_t>(data_save_len));
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

ErrorCode MotorController::refresh_motor_status(uint16_t motor_id) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
    }
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0xCC, 0, 0, 0, 0, 0};
    send_data(0x7FF, data_buf);
    recv();
    return ErrorCode::SUCCESS;
}


void MotorController::write_motor_param(uint16_t motor_id, DMVariable rid, float data) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return;
    }
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


void MotorController::read_rid_param(uint16_t motor_id, DMVariable rid) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return;
    }
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0x33, static_cast<uint8_t>(rid), 0, 0, 0, 0};
    send_data(0x7FF, data_buf);
}


ErrorCode MotorController::switch_control_mode(uint16_t motor_id, ControlType mode) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
    }
    const int max_retries = 20;
    const int retry_interval_ms = 100;
    DMVariable rid = DMVariable::CTRL_MODE;

    write_motor_param(motor_id, rid, static_cast<float>(static_cast<int>(mode)));

    for (int i = 0; i < max_retries; ++i) {
        // WARNING: This blocking sleep is not suitable for real-time applications.
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
        recv_set_param_data();
        if (motor->has_param(rid)) {
            if (static_cast<int>(motor->get_param(rid)) == static_cast<int>(mode)) {
                return ErrorCode::SUCCESS;
            } else {
                return ErrorCode::WRITE_FAILED;
            }
        }
    }
    return ErrorCode::ERROR_COMMUNICATION_TIMEOUT;
}

ErrorCode MotorController::save_motor_param(uint16_t motor_id) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return ErrorCode::MOTOR_NOT_FOUND;
    }
    uint16_t can_id_l = motor->get_slave_id() & 0xff;
    uint16_t can_id_h = (motor->get_slave_id() >> 8) & 0xff;
    std::array<uint8_t, 8> data_buf = {static_cast<uint8_t>(can_id_l), static_cast<uint8_t>(can_id_h), 0xAA, 0, 0, 0, 0, 0};

    disable_motor(motor_id);
    send_data(0x7FF, data_buf);
    // WARNING: This blocking sleep is not suitable for real-time applications.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return ErrorCode::SUCCESS;
}

float MotorController::read_motor_param(uint16_t motor_id, DMVariable rid) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return 0.0f; // Return default value if not found
    }
    const int max_retries = 20;
    const int retry_interval_ms = 50;

    read_rid_param(motor_id, rid);
    for (int i = 0; i < max_retries; ++i) {
        // WARNING: This blocking sleep is not suitable for real-time applications.
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
        recv_set_param_data();
        if (motor->has_param(rid)) {
            return motor->get_param(rid);
        }
    }
    return 0.0f; // Return default value on timeout
}

bool MotorController::change_motor_param(uint16_t motor_id, DMVariable rid, float data) {
    Motor* motor = find_motor_by_slave_id(motor_id);
    if (motor == nullptr) {
        return false;
    }
    const int max_retries = 20;
    const int retry_interval_ms = 50;

    write_motor_param(motor_id, rid, data);
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
        // WARNING: This blocking sleep is not suitable for real-time applications.
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
    }
    return false;
}

} // namespace dm_motor_sdk