#include "UMDLoopCANProtocol.hpp"
#include <cstdio>
#include <cstdlib>

using namespace Protocol;

static int g_failures = 0;

#define CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::fprintf(stderr, "FAIL [line %d]: %s\n", __LINE__, #expr); \
            ++g_failures; \
        } \
    } while (false)

static void test_message_type_dispatch() {
    CanFrame f;

    f.id = POWER_PCB_C::ID;
    CHECK(getMessageType(f) == MessageType::POWER_PCB_C);

    f.id = SCIENCE_SERVO_PCB_C::ID;
    CHECK(getMessageType(f) == MessageType::SCIENCE_SERVO_PCB_C);

    f.id = SCIENCE_SERVO_PCB_R::ID;
    CHECK(getMessageType(f) == MessageType::SCIENCE_SERVO_PCB_R);

    f.id = SWERVE_PCB_C::ID;
    CHECK(getMessageType(f) == MessageType::SWERVE_PCB_C);

    f.id = 0xDEAD;
    CHECK(getMessageType(f) == MessageType::Unknown);

    std::puts("test_message_type_dispatch done");
}

static void test_power_pcb_flat_roundtrip() {
    POWER_PCB_C out;
    out.cmd = 0x11;

    CanFrame frame = encode(out);
    CHECK(frame.id  == POWER_PCB_C::ID);   // 0x10
    CHECK(frame.dlc == POWER_PCB_C::DLC);  // 3
    CHECK(frame.data[0] == 0x11);          // cmd in byte 0

    POWER_PCB_C in = decode<POWER_PCB_C>(frame);
    CHECK(in.cmd == 0x11);

    std::puts("test_power_pcb_flat_roundtrip done");
}

static void test_science_servo_velocity_roundtrip() {
    for (uint8_t port = 0; port < 8; ++port) {
        int16_t vel = static_cast<int16_t>(-500 + port * 150);  // mix of negative and positive

        SCIENCE_SERVO_PCB_C_servo_velocity_target_t payload;
        payload.port_id = port;
        payload.servo_velocity_target = vel;

        CanFrame frame = encode(payload);

        CHECK(frame.id  == SCIENCE_SERVO_PCB_C::ID);   // 0x80
        CHECK(frame.dlc == 3);
        CHECK(frame.data[0] == static_cast<uint8_t>(0x30 + port));  // cmd byte
        // little-endian payload
        CHECK(frame.data[1] == static_cast<uint8_t>(vel & 0xFF));
        CHECK(frame.data[2] == static_cast<uint8_t>((vel >> 8) & 0xFF));

        // decode payload
        auto decoded = decode<SCIENCE_SERVO_PCB_C_servo_velocity_target_t>(frame);
        CHECK(decoded.port_id == port);
        CHECK(decoded.servo_velocity_target == vel);
    }
    std::puts("test_science_servo_velocity_roundtrip done");
}

static void test_science_servo_position_roundtrip() {
    for (uint8_t port = 0; port < 8; ++port) {
        int16_t pos = static_cast<int16_t>(port * 1000 - 3500);

        SCIENCE_SERVO_PCB_C_servo_position_target_t payload;
        payload.port_id = port;
        payload.servo_position_target = pos;

        CanFrame frame = encode(payload);

        CHECK(frame.id  == SCIENCE_SERVO_PCB_C::ID);
        CHECK(frame.dlc == 3);
        CHECK(frame.data[0] == static_cast<uint8_t>(0x20 + port));
        CHECK(frame.data[1] == static_cast<uint8_t>(pos & 0xFF));
        CHECK(frame.data[2] == static_cast<uint8_t>((pos >> 8) & 0xFF));

        auto decoded = decode<SCIENCE_SERVO_PCB_C_servo_position_target_t>(frame);
        CHECK(decoded.port_id == port);
        CHECK(decoded.servo_position_target == pos);
    }
    std::puts("test_science_servo_position_roundtrip done");
}

static void test_science_servo_bool_commands() {
    for (uint8_t port = 0; port < 8; ++port) {
        SCIENCE_SERVO_PCB_C_servo_state_req_event_t state_payload;
        state_payload.port_id = port;
        state_payload.servo_state_req_event = true;
        CanFrame state_frame  = encode(state_payload);

        SCIENCE_SERVO_PCB_C_servo_status_req_event_t status_payload;
        status_payload.port_id = port;
        status_payload.servo_status_req_event = true;
        CanFrame status_frame = encode(status_payload);

        CHECK(state_frame.data[0]  == static_cast<uint8_t>(0x40 + port));
        CHECK(status_frame.data[0] == static_cast<uint8_t>(0x50 + port));

        auto state_decoded = decode<SCIENCE_SERVO_PCB_C_servo_state_req_event_t>(state_frame);
        CHECK(state_decoded.port_id == port);
        CHECK(state_decoded.servo_state_req_event == true);

        auto status_decoded = decode<SCIENCE_SERVO_PCB_C_servo_status_req_event_t>(status_frame);
        CHECK(status_decoded.port_id == port);
        CHECK(status_decoded.servo_status_req_event == true);
    }
    std::puts("test_science_servo_bool_commands done");
}

static void test_science_servo_pos_response_roundtrip() {
    for (uint8_t port = 0; port < 8; ++port) {
        SCIENCE_SERVO_PCB_R_pos_resp_t payload;
        payload.port_id = port;
        payload.servo_position_pos_resp = static_cast<int16_t>(port * 200 - 700);
        payload.servo_velocity_pos_resp = static_cast<int16_t>(port * 50  - 100);

        CanFrame frame = encode(payload);

        CHECK(frame.id  == SCIENCE_SERVO_PCB_R::ID);   // 0x81
        CHECK(frame.dlc == 5);
        CHECK(frame.data[0] == static_cast<uint8_t>(0x20 + port));
        // position little-endian at bytes 1-2
        CHECK(frame.data[1] == static_cast<uint8_t>(payload.servo_position_pos_resp & 0xFF));
        CHECK(frame.data[2] == static_cast<uint8_t>((payload.servo_position_pos_resp >> 8) & 0xFF));
        // velocity little-endian at bytes 3-4
        CHECK(frame.data[3] == static_cast<uint8_t>(payload.servo_velocity_pos_resp & 0xFF));
        CHECK(frame.data[4] == static_cast<uint8_t>((payload.servo_velocity_pos_resp >> 8) & 0xFF));

        auto decoded = decode<SCIENCE_SERVO_PCB_R_pos_resp_t>(frame);
        CHECK(decoded.port_id == port);
        CHECK(decoded.servo_position_pos_resp == payload.servo_position_pos_resp);
        CHECK(decoded.servo_velocity_pos_resp == payload.servo_velocity_pos_resp);
    }
    std::puts("test_science_servo_pos_response_roundtrip done");
}

static void test_swerve_pcb_mux_commands() {
    for (uint8_t port = 0; port < 2; ++port) {
        int16_t pos = static_cast<int16_t>(port * 4500 - 2000);
        int16_t vel = static_cast<int16_t>(port * 300  -  100);

        SWERVE_PCB_C_servo_position_target_t pos_payload{port, pos};
        SWERVE_PCB_C_servo_velocity_target_t vel_payload{port, vel};

        CanFrame pos_frame = encode(pos_payload);
        CanFrame vel_frame = encode(vel_payload);

        CHECK(pos_frame.id == SWERVE_PCB_C::ID);
        CHECK(vel_frame.id == SWERVE_PCB_C::ID);

        auto pos_decoded = decode<SWERVE_PCB_C_servo_position_target_t>(pos_frame);
        CHECK(pos_decoded.port_id == port);
        CHECK(pos_decoded.servo_position_target == pos);

        auto vel_decoded = decode<SWERVE_PCB_C_servo_velocity_target_t>(vel_frame);
        CHECK(vel_decoded.port_id == port);
        CHECK(vel_decoded.servo_velocity_target == vel);
    }
    std::puts("test_swerve_pcb_mux_commands done");
}

static void test_boundary_values() {
    {
        SCIENCE_SERVO_PCB_C_servo_velocity_target_t payload{0, INT16_MIN};
        CanFrame f = encode(payload);
        auto decoded = decode<SCIENCE_SERVO_PCB_C_servo_velocity_target_t>(f);
        CHECK(decoded.servo_velocity_target == INT16_MIN);
    }
    {
        SCIENCE_SERVO_PCB_C_servo_velocity_target_t payload{7, INT16_MAX};
        CanFrame f = encode(payload);
        auto decoded = decode<SCIENCE_SERVO_PCB_C_servo_velocity_target_t>(f);
        CHECK(decoded.servo_velocity_target == INT16_MAX);
        CHECK(decoded.port_id == 7);
    }
    // zero
    {
        SCIENCE_SERVO_PCB_C_servo_position_target_t payload{3, 0};
        CanFrame f = encode(payload);
        auto decoded = decode<SCIENCE_SERVO_PCB_C_servo_position_target_t>(f);
        CHECK(decoded.servo_position_target == 0);
        CHECK(f.data[1] == 0);
        CHECK(f.data[2] == 0);
    }
    std::puts("test_boundary_values done");
}

int main() {
    test_message_type_dispatch();
    test_power_pcb_flat_roundtrip();
    test_science_servo_velocity_roundtrip();
    test_science_servo_position_roundtrip();
    test_science_servo_bool_commands();
    test_science_servo_pos_response_roundtrip();
    test_swerve_pcb_mux_commands();
    test_boundary_values();

    if (g_failures == 0) {
        std::puts("\nAll tests passed.");
        return 0;
    }
    std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
    return 1;
}
