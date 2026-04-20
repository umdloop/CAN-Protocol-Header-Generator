#include "UMDLoopCANProtocol.hpp"
#include <cstdio>
#include <optional>
#include <vector>

using namespace Protocol;

static int g_failures = 0;

#define CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::fprintf(stderr, "FAIL [line %d]: %s\n", __LINE__, #expr); \
            ++g_failures; \
        } \
    } while (false)

static void test_servo() {
    Servo::PositionTarget target{4500};
    CanFrame frame = Servo::encode_position_target(Subsystem::ScienceServo, 2, target);
    CHECK(frame.id == 0x80);
    CHECK(frame.data[0] == 0x22);
    auto decoded = Servo::decode_position_target(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == 4500);
    std::puts("test_servo passed");
}

static void test_dc_motor() {
    DcMotor::Status stat{10};
    CanFrame frame = DcMotor::encode_status(Subsystem::EndEffector, 1, stat);
    CHECK(frame.id == 0x61);
    CHECK(frame.data[0] == 0x51);
    auto decoded = DcMotor::decode_status(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == 10);
    std::puts("test_dc_motor passed");
}

static void test_stepper() {
    Stepper::RelPosTarget rel{1000};
    CanFrame frame = Stepper::encode_rel_pos_target(Subsystem::ScienceStepper, 0, rel);
    CHECK(frame.id == 0x90);
    CHECK(frame.data[0] == 0x20); 
    auto decoded = Stepper::decode_rel_pos_target(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == 1000);
    std::puts("test_stepper passed");
}

static void test_kill_system() {
    Power::MainPower kill_cmd{true};
    CanFrame frame = Power::encode_main_power(Subsystem::Power, 0, kill_cmd);
    CHECK(frame.id == 0x10);
    auto decoded = Power::decode_main_power(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == true);
    std::puts("test_kill_system passed");
}

static void test_spectroscopy() {
    Ccd::BinaryCcdStream stream{123, 0xABCDE};
    CanFrame frame = Ccd::encode_binary_ccd_stream(Subsystem::Spectroscopy, 0, stream);
    CHECK(frame.id == 0x102);
    auto decoded = Ccd::decode_binary_ccd_stream(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.byte_frame_id_0 == 123);
    CHECK(decoded->data.binary_measurement_0 == 0xABCDE);
    std::puts("test_spectroscopy passed");
}

static void test_diode() {
    SensorDiode::Intensity val{200};
    CanFrame frame = SensorDiode::encode_intensity(Subsystem::Fluorometry, 0, val);
    CHECK(frame.id == 0x111);
    auto decoded = SensorDiode::decode_intensity(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == 200);
    std::puts("test_diode passed");
}

static void test_laser() {
    Laser::Command cmd{true};
    CanFrame frame = Laser::encode_command(Subsystem::EndEffector, 0, cmd);
    CHECK(frame.id == 0x60);
    CHECK(frame.data[0] == 0x2a);
    auto decoded = Laser::decode_command(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == true);
    std::puts("test_laser passed");
}

static void test_led() {
    Led::RgbCmd rgb{0, 0, 255};
    CanFrame frame = Led::encode_rgb_cmd(Subsystem::DriveLed, 0, rgb);
    CHECK(frame.id == 0x120);
    auto decoded = Led::decode_rgb_cmd(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.red == 255);
    std::puts("test_led passed");
}

static void test_limit_switch() {
    LimitSwitch::ReqEvent req{true};
    CanFrame frame = LimitSwitch::encode_req_event(Subsystem::BaseArm, 0, req);
    CHECK(frame.id == 0x130);
    auto decoded = LimitSwitch::decode_req_event(frame);
    CHECK(decoded.has_value());
    CHECK(decoded->data.value == true);
    std::puts("test_limit_switch passed");
}

int main() {
    test_servo();
    test_dc_motor();
    test_stepper();
    test_kill_system();
    test_spectroscopy();
    test_diode();
    test_laser();
    test_led();
    test_limit_switch();

    if (g_failures == 0) {
        std::puts("\nAll comprehensive tests passed.");
        return 0;
    }
    std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
    return 1;
}
