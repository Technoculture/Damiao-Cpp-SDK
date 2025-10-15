#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "dm_motor_sdk/protocol_utils.h"

TEST_CASE("Protocol Utils float_to_uint", "[protocol_utils]") {
    SECTION("Basic conversion") {
        float val = 5.0f;
        float min = 0.0f;
        float max = 10.0f;
        int bits = 16;
        uint16_t result = dm_motor_sdk::float_to_uint(val, min, max, bits);
        REQUIRE(result == 32767);
    }

    SECTION("Clamping high") {
        float val = 15.0f;
        float min = 0.0f;
        float max = 10.0f;
        int bits = 16;
        uint16_t result = dm_motor_sdk::float_to_uint(val, min, max, bits);
        REQUIRE(result == 65535);
    }

    SECTION("Clamping low") {
        float val = -5.0f;
        float min = 0.0f;
        float max = 10.0f;
        int bits = 16;
        uint16_t result = dm_motor_sdk::float_to_uint(val, min, max, bits);
        REQUIRE(result == 0);
    }
}

TEST_CASE("Protocol Utils uint_to_float", "[protocol_utils]") {
    SECTION("Basic conversion") {
        uint16_t val = 32767;
        float min = 0.0f;
        float max = 10.0f;
        int bits = 16;
        float result = dm_motor_sdk::uint_to_float(val, min, max, bits);
        REQUIRE(result == Catch::Approx(5.0f).margin(0.001));
    }
}