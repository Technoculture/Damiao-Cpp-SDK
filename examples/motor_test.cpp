#include "dm_motor_sdk/motor.h"
#include "dm_motor_sdk/motor_controller.h"
#include "dm_motor_sdk/serial_device.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <cassert>

void print_motor_state(dm_motor_sdk::Motor* motor, const char* name) {
    std::cout << name << ": POS: " << motor->get_position()
              << " VEL: " << motor->get_velocity()
              << " TORQUE: " << motor->get_torque() << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return 1;
    }
    const char* port_name = argv[1];

    // 1. Create motor and controller objects
    dm_motor_sdk::Motor motor1(dm_motor_sdk::DMMotorType::DM4310, 0x01, 0x11);
    dm_motor_sdk::Motor motor2(dm_motor_sdk::DMMotorType::DM4310, 0x05, 0x15);

    dm_motor_sdk::SerialDevice serial_device(port_name, 921600);
    if (serial_device.open_device() != dm_motor_sdk::SerialErrorCode::SUCCESS) {
        std::cerr << "Failed to open serial port " << port_name << std::endl;
        return -1;
    }

    dm_motor_sdk::MotorController motor_controller(&serial_device);
    motor_controller.add_motor(&motor1);
    motor_controller.add_motor(&motor2);

    // 2. Initial configuration and parameter check
    if (motor_controller.switch_control_mode(&motor1, dm_motor_sdk::ControlType::POS_VEL) == dm_motor_sdk::ControllerErrorCode::SUCCESS) {
        std::cout << "Motor1: Switched to POS_VEL mode successfully" << std::endl;
    }
    if (motor_controller.switch_control_mode(&motor2, dm_motor_sdk::ControlType::VEL) == dm_motor_sdk::ControllerErrorCode::SUCCESS) {
        std::cout << "Motor2: Switched to VEL mode successfully" << std::endl;
    }

    std::cout << "Motor1 Gr: " << motor_controller.read_motor_param(&motor1, dm_motor_sdk::DMVariable::Gr) << std::endl;
    std::cout << "Motor1 PMAX: " << motor_controller.read_motor_param(&motor1, dm_motor_sdk::DMVariable::PMAX) << std::endl;

    // 3. Enable motors
    motor_controller.save_motor_param(&motor1);
    motor_controller.save_motor_param(&motor2);
    motor_controller.enable_motor(&motor1);
    motor_controller.enable_motor(&motor2);

    // 4. Main control loop
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        auto current_time = std::chrono::high_resolution_clock::now();
        double t = std::chrono::duration<double>(current_time - start_time).count();
        float q = std::sin(static_cast<float>(t));

        motor_controller.control_pos_vel(&motor1, q * 8.0f, 30.0f);
        motor_controller.control_vel(&motor2, 8.0f * q);

        // Optional: print state periodically
        if (i % 100 == 0) {
            print_motor_state(&motor1, "Motor1");
            print_motor_state(&motor2, "Motor2");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 5. Disable motors and close serial port
    motor_controller.disable_motor(&motor1);
    motor_controller.disable_motor(&motor2);
    serial_device.close_device();

    std::cout << "Test finished." << std::endl;

    return 0;
}