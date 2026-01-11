# ESP32 Gripper Controller

An ESP32-based firmware for a robotic gripper system with stepper motor control and PS4 gamepad interface. The gripper automatically sorts objects by size using pressure feedback and intelligent motion control.

## Features

- **Delay-free state machine architecture** running synchronously in the main loop
- **Non-blocking stepper motor control** using the MobaTools library with interrupt-driven step commands
- **Software-based Position Profile Mode (PPM)** controller for smooth acceleration and deceleration
- **Multi-modal control interfaces**:
  - Serial command interface for debugging and monitoring
  - Bluetooth PS4 gamepad control with LED and haptic feedback
- **Software step counter** tracking relative motor position across varying microstepping modes
- **PID-controlled gripping** with pressure sensor feedback
- **Automatic object sorting** based on size detection
- **Time-critical operations** using elapsed-time checks for consistent loop timing

## Hardware Requirements

- ESP32-DOIT-DEVKIT-V1 board
- Stepper motor (200 steps/revolution)
- Stepper motor driver (MS1/MS2 microstepping control)
- Pressure sensor (analog)
- Servo motor for sorting mechanism
- PS4 controller (for wireless operation)
- LEDs and buzzer for status indication

### Pin Configuration

See `src/constants.h` for detailed pin assignments:
- Stepper: DIR (14), STEP (27), ENABLE (26), MS1 (33), MS2 (25)
- Servo: PIN (12)
- Pressure sensor: PIN (34)
- Status LEDs: Pins 21, 19, 18, 5, 17, 16
- Buzzer: PIN (32)

## Software Architecture

The firmware uses a **synchronous finite state machine** that processes inputs and updates outputs without blocking delays. Key states include:
- **IDLE/ARM**: Ready state, holding position
- **FEEL**: Initial contact and ripeness detection
- **GRIP**: PID-controlled gripping with pressure feedback
- **SORT**: Size classification based on gripper position
- **RELEASE**: Object release with timeout

Motor control features:
- Software step counter compensates for microstepping mode changes (2x, 4x, 8x, 16x)
- Trapezoidal motion profile ensures smooth acceleration/deceleration
- Position limits prevent mechanical damage
- State-retention logic for button inputs prevents double-triggering

## Installation

### Prerequisites

- [PlatformIO](https://platformio.org/) installed
- ESP32 board support in PlatformIO

### Build and Upload

1. Clone this repository:
   ```bash
   git clone https://github.com/NicolAlex/2025_GRIPPER_49.git
   cd 2025_GRIPPER_49
   ```

2. Build the project:
   ```bash
   pio run -e esp32doit-devkit-v1
   ```

3. Upload to your ESP32:
   ```bash
   pio run -t upload -e esp32doit-devkit-v1
   ```

4. Monitor serial output:
   ```bash
   pio device monitor -e esp32doit-devkit-v1
   ```

## PS4 Controller Setup

1. Pair your PS4 controller using a pairing tool (see [PS4-esp32 documentation](lib/PS4-esp32/README.md))
2. Update the MAC address in `src/main.cpp` line 280:
   ```cpp
   PS4.begin("c8:c9:a3:c7:8d:7e"); // Replace with your controller's MAC
   ```
3. Press the PS button to connect

### PS4 Button Mapping

- **R1**: Arm/disarm gripper
- **Circle**: Execute grip action
- **Up/Down**: Close/open gripper (quick move)
- **Left/Right**: Uniform move (slow, precise control)
- **Triangle**: Print status
- **Options**: Calibrate gripper
- **Share**: Toggle verbose mode
- **Cross**: Release object (during release state)

LED colors indicate gripper state, and rumble provides haptic feedback during critical operations.

## Serial Commands

Access the serial terminal at 115200 baud to use these commands:

### Basic Control
- `cal` - Calibrate gripper origin
- `arm` - Arm gripper (hold position)
- `disarm` - Disarm gripper (idle)
- `status` - Print current status

### Configuration
- `set fpos <value>` - Set target position (steps)
- `set v <value>` - Set speed (RPM)
- `set ms <value>` - Set microstepping (2, 4, 8, or 16)
- `set vmax <value>` - Set max speed (RPM)
- `set vmin <value>` - Set min speed (RPM)
- `set accel <value>` - Set acceleration (steps/sec²)

### Monitoring
- `get pos` - Get current position
- `get v` - Get current speed
- `get ms` - Get microstepping mode
- `get fpos` - Get target position
- `get state` - Get FSM state
- `verbose <on/off>` - Toggle verbose output

## Project Structure

```
.
├── platformio.ini          # PlatformIO configuration
├── src/
│   ├── main.cpp           # Main program entry, PS4 integration
│   ├── gripper.cpp        # Gripper FSM and motor control
│   ├── gripper.h          # Gripper class and function declarations
│   └── constants.h        # Pin definitions and configuration
├── lib/
│   └── PS4-esp32/         # PS4 controller library
└── include/               # Additional headers
```

## Acknowledgments

This project uses the following open-source libraries:

- **[MobaTools](https://github.com/MicroBahner/MobaTools)** by Franz-Peter Müller - Arduino library for non-blocking stepper and servo control with interrupt-driven step generation. Licensed under GPL v3.
- **[PS4-esp32](https://github.com/aed3/PS4-esp32)** by aed3 - PS4 controller library for ESP32, based on esp32-ps3 by Jeffrey Pernis. Licensed under MIT.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Copyright (c) 2025 Nicolas Alexandre
