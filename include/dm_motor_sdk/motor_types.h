#ifndef DM_MOTOR_SDK_MOTOR_TYPES_H
#define DM_MOTOR_SDK_MOTOR_TYPES_H

#include <cstdint>
#include <array>

namespace dm_motor_sdk {

enum class DMMotorType : int {
    DM4310 = 0,
    DM4310_48V = 1,
    DM4340 = 2,
    DM4340_48V = 3,
    DM6006 = 4,
    DM8006 = 5,
    DM8009 = 6,
    DM10010L = 7,
    DM10010 = 8,
    DMH3510 = 9,
    DMH6215 = 10,
    DMG6220 = 11
};

enum class DMVariable : int {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81
};

enum class ControlType : int {
    MIT = 1,
    POS_VEL = 2,
    VEL = 3,
    Torque_Pos = 4
};

struct MotorLimits {
    float p_max;
    float v_max;
    float t_max;
};

// Corresponds to the Limit_Param in the Python code
static const std::array<MotorLimits, 12> LIMIT_PARAM = {{
    {12.5, 30, 10},   // DM4310
    {12.5, 50, 10},   // DM4310_48V
    {12.5, 8, 28},    // DM4340
    {12.5, 10, 28},   // DM4340_48V
    {12.5, 45, 20},   // DM6006
    {12.5, 45, 40},   // DM8006
    {12.5, 45, 54},   // DM8009
    {12.5, 25, 200},  // DM10010L
    {12.5, 20, 200},  // DM10010
    {12.5, 280, 1},   // DMH3510
    {12.5, 45, 10},   // DMG62150
    {12.5, 45, 10}    // DMH6220
}};

} // namespace dm_motor_sdk

#endif // DM_MOTOR_SDK_MOTOR_TYPES_H