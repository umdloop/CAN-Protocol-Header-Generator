# UMD Loop DBC-to-CAN Generator

This tool takes DBC files and spits out a header file that lets you encode and decode messages easily.

## Build Instructions

### Prerequisites
- CMake 3.14+
- C++17

### Building & Generating
```bash
mkdir build && cd build
cmake ..
make
```
The build process automatically runs the generator. You can find the resulting API at:
`build/generated/UMDLoopCANProtocol.hpp`

### Running the Example/Tests
To ensure everything is working correctly:
```bash
make protocol-test
./protocol-test
```

---

## Adding New Devices

To make the generation work, you must follow some conventions:

### 1. Device Prefixes
The generator uses prefixes to decide which namespace a signal belongs to. If you want a signal to appear in the `Servo` namespace, it must start with `servo_`.

**Currently supported prefixes:**
`dc_motor`, `servo`, `stepper`, `laser`, `led`, `rotary_encoder`, `limit_switch`, `diode`, `ccd`, `spectroscopy`, `fluorometry`, `power`, `swerve`, `arm`, `kill`, `pcb`.

### 2. Adding a New Device Type
If you are adding a completely new type of hardware (e.g., a `lidar`), follow these steps:
1. Open `src/generator.cpp`.
2. Locate the `devices` vector inside the `parseDeviceCommand` function.
3. Add `"lidar"` to the list.
4. Rebuild the project.

### 3. Port Multiplexing (Suffixes)
For PCBs that have multiple identical ports (like a Servo board with 8 slots), name your signals with a port suffix (e.g., `servo_position_target_0` through `_7`). 
- The generator will collapse these into a single function.
- In C++, you’ll call `Servo::encode_position_target(pcb, port, data)`, where `port` is just an integer.

### 4. Grouping Signals
For "flat" messages (no multiplexer), the generator groups every signal inside that CAN frame into a single C++ struct.
