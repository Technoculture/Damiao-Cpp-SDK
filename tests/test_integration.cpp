#include <catch2/catch_test_macros.hpp>
#include "dm_motor_sdk/motor.h"
#include "dm_motor_sdk/motor_controller.h"
#include "mock_serial_device.h"
#include "dm_motor_sdk/logger.h"
#include <fmt/core.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <cassert>
#include <memory>

TEST_CASE("Motor Controller Integration Test with Mock Serial", "[integration]") {
    dm_motor_sdk::Logger::initialize();

    // 1. Create motor and controller objects
    auto motor1 = std::make_unique<dm_motor_sdk::Motor>(dm_motor_sdk::DMMotorType::DM4310, 0x01, 0x11);
    uint16_t motor1_id = motor1->get_slave_id();

    MockSerialDevice mock_serial_device;
    dm_motor_sdk::MotorController motor_controller(&mock_serial_device);
    motor_controller.add_motor(std::move(motor1));

    // 2. Enable motor
    REQUIRE(motor_controller.enable_motor(motor1_id) == dm_motor_sdk::ErrorCode::SUCCESS);
    REQUIRE(mock_serial_device.write_buffer.size() > 0);
}