#include "dm_motor_sdk/motor.h"
#include <cstring> // For memset

namespace dm_motor_sdk {

Motor::Motor(DMMotorType motor_type, uint16_t slave_id, uint16_t master_id) {
    motor_type_ = motor_type;
    slave_id_ = slave_id;
    master_id_ = master_id;
    state_q_ = 0.0f;
    state_dq_ = 0.0f;
    state_tau_ = 0.0f;

    // Initialize parameter arrays
    for (int i = 0; i < MAX_PARAMS; ++i) {
        params_[i] = 0.0f;
        params_set_[i] = false;
    }
}

void Motor::recv_data(float q, float dq, float tau) {
    state_q_ = q;
    state_dq_ = dq;
    state_tau_ = tau;
}

float Motor::get_position() const {
    return state_q_;
}

float Motor::get_velocity() const {
    return state_dq_;
}

float Motor::get_torque() const {
    return state_tau_;
}

void Motor::set_param(DMVariable rid, float value) {
    int index = static_cast<int>(rid);
    if (index >= 0 && index < MAX_PARAMS) {
        params_[index] = value;
        params_set_[index] = true;
    }
}

float Motor::get_param(DMVariable rid) const {
    int index = static_cast<int>(rid);
    if (index >= 0 && index < MAX_PARAMS && params_set_[index]) {
        return params_[index];
    }
    return 0.0f; // Return a default value if not found
}

bool Motor::has_param(DMVariable rid) const {
    int index = static_cast<int>(rid);
    if (index >= 0 && index < MAX_PARAMS) {
        return params_set_[index];
    }
    return false;
}

DMMotorType Motor::get_motor_type() const {
    return motor_type_;
}

uint16_t Motor::get_slave_id() const {
    return slave_id_;
}

uint16_t Motor::get_master_id() const {
    return master_id_;
}

} // namespace dm_motor_sdk