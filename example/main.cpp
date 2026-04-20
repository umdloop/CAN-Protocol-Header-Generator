#include "UMDLoopCANProtocol.hpp"
#include <cstdio>
#include <optional>

using namespace Protocol;

static int g_failures = 0;

#define CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::fprintf(stderr, "FAIL [line %d]: %s\n", __LINE__, #expr); \
            ++g_failures; \
        } \
    } while (false)

static void test_servo_position_roundtrip() {
    Servo::PositionTarget target{180};
    // ScienceServo, Port 3 -> CAN ID 0x80, Mux 0x20 + 3 = 0x23
    CanFrame frame = Servo::encode_position_target(Subsystem::ScienceServo, 3, target);
    
    CHECK(frame.id == 0x80);
    CHECK(frame.data[0] == 0x23);
    
    auto decoded = Servo::decode_position_target(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->port == 3);
    CHECK(decoded->data.value == 180);
    
    std::puts("test_servo_position_roundtrip done");
}

static void test_servo_position_multi_pcb() {
    Servo::PositionTarget target{90};
    
    // Swerve, Port 0 -> CAN ID 0x140, Mux 0x20 + 0 = 0x20
    CanFrame frame_swerve = Servo::encode_position_target(Subsystem::Swerve, 0, target);
    CHECK(frame_swerve.id == 0x140);
    CHECK(frame_swerve.data[0] == 0x20);
    
    auto decoded_swerve = Servo::decode_position_target(frame_swerve);
    CHECK(decoded_swerve.has_value());
    CHECK(decoded_swerve->port == 0);
    CHECK(decoded_swerve->data.value == 90);

    // BaseArm, Port 2 -> CAN ID 0x130, Mux 0x20 + 2 = 0x22
    CanFrame frame_arm = Servo::encode_position_target(Subsystem::BaseArm, 2, target);
    CHECK(frame_arm.id == 0x130);
    CHECK(frame_arm.data[0] == 0x22);

    auto decoded_arm = Servo::decode_position_target(frame_arm);
    CHECK(decoded_arm.has_value());
    CHECK(decoded_arm->port == 2);
    
    std::puts("test_servo_position_multi_pcb done");
}

static void test_dc_motor_status() {
    DcMotor::Status stat{1}; 
    
    CanFrame frame = DcMotor::encode_status(Subsystem::ScienceDcMotor, 0, stat);
    CHECK(frame.id == 0x71);
    
    auto decoded = DcMotor::decode_status(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == 1);
    
    std::puts("test_dc_motor_status done");
}

int main() {
    test_servo_position_roundtrip();
    test_servo_position_multi_pcb();
    test_dc_motor_status();

    if (g_failures == 0) {
        std::puts("\nAll tests passed.");
        return 0;
    }
    std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
    return 1;
}
