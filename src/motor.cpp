#include "dm_motor_sdk/motor.h"
#include <cstring> // For memset

namespace dm_motor_sdk {

Motor::Motor(DMMotorType new_motor_type, uint16_t new_slave_id, uint16_t new_master_id) {
    this->motor_type = new_motor_type;
    this->slave_id = new_slave_id;
    this->master_id = new_master_id;
    this->state_q = 0.0f;
    this->state_dq = 0.0f;
    this->state_tau = 0.0f;

    // Initialize parameter arrays
    params.fill(0.0f);
    params_set.fill(false);
}

void Motor::recv_data(float q, float dq, float tau) {
    state_q = q;
    state_dq = dq;
    state_tau = tau;
}

float Motor::get_position() const {
    return state_q;
}

float Motor::get_velocity() const {
    return state_dq;
}

float Motor::get_torque() const {
    return state_tau;
}

void Motor::set_param(DMVariable rid, float value) {
    size_t index = static_cast<size_t>(rid);
    if (index < MAX_PARAMS) {
        params[index] = value;
        params_set[index] = true;
    }
}

float Motor::get_param(DMVariable rid) const {
    size_t index = static_cast<size_t>(rid);
    if (index < MAX_PARAMS && params_set[index]) {
        return params[index];
    }
    return 0.0f; // Return a default value if not found
}

bool Motor::has_param(DMVariable rid) const {
    size_t index = static_cast<size_t>(rid);
    if (index < MAX_PARAMS) {
        return params_set[index];
    }
    return false;
}

DMMotorType Motor::get_motor_type() const {
    return motor_type;
}

uint16_t Motor::get_slave_id() const {
    return slave_id;
}

uint16_t Motor::get_master_id() const {
    return master_id;
}

} // namespace dm_motor_sdk