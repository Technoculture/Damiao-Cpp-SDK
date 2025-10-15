#ifndef DM_MOTOR_SDK_MOTOR_H
#define DM_MOTOR_SDK_MOTOR_H

#include "motor_types.h"

namespace dm_motor_sdk {

const int MAX_PARAMS = 82; // Max value in DMVariable enum

class Motor {
public:
    Motor(DMMotorType motor_type, uint16_t slave_id, uint16_t master_id);

    void recv_data(float q, float dq, float tau);

    float get_position() const;
    float get_velocity() const;
    float get_torque() const;

    // Using a C-style array for parameters as per style guide
    // A simple approach to mimic the dictionary behavior for a fixed set of keys.
    void set_param(DMVariable rid, float value);
    float get_param(DMVariable rid) const;
    bool has_param(DMVariable rid) const;

    DMMotorType get_motor_type() const;
    uint16_t get_slave_id() const;
    uint16_t get_master_id() const;

private:
    float state_q_;
    float state_dq_;
    float state_tau_;
    uint16_t slave_id_;
    uint16_t master_id_;
    DMMotorType motor_type_;

    // Fixed-size array to store parameters, indexed by DMVariable enum
    float params_[MAX_PARAMS];
    bool params_set_[MAX_PARAMS];
};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_MOTOR_H