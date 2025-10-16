#ifndef DM_MOTOR_SDK_MOTOR_H
#define DM_MOTOR_SDK_MOTOR_H

#include "motor_types.h"
#include <array>

namespace dm_motor_sdk {

const int MAX_PARAMS = 82; // Max value in DMVariable enum

class Motor {
public:
    Motor(DMMotorType motor_type, uint16_t slave_id, uint16_t master_id);

    void recv_data(float q, float dq, float tau);

    float get_position() const;
    float get_velocity() const;
    float get_torque() const;

    void set_param(DMVariable rid, float value);
    float get_param(DMVariable rid) const;
    bool has_param(DMVariable rid) const;

    DMMotorType get_motor_type() const;
    uint16_t get_slave_id() const;
    uint16_t get_master_id() const;

private:
    float state_q;
    float state_dq;
    float state_tau;
    uint16_t slave_id;
    uint16_t master_id;
    DMMotorType motor_type;

    std::array<float, MAX_PARAMS> params;
    std::array<bool, MAX_PARAMS> params_set;
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_MOTOR_H