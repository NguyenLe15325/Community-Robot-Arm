# Community-Robot-Arm

Minimal, practical firmware for a 3-DOF community robot arm (Florin Tobler design lineage), running on Arduino Nano/Uno.

This project focuses on:
- reliable joint and Cartesian motion
- configurable endstop homing
- simple serial G-code interface
- low overhead for resource-limited boards

## Status

Stable stopping point for this project version.

Current behavior summary:
- Manual startup (no auto-home on boot)
- Endstop homing supported with calibrated offsets
- Software-only homing fallback supported when no endstops are installed
- Emergency stop command available (`M112` and `!` alias)

## Repository Layout

```
Community-Robot-Arm/
|- firmware/
|  |- RobotArm/
|     |- RobotArm.ino
|     |- Config_Pinout.h
|     |- Config_Robot.h
|     |- GCode.h / GCode.cpp
|     |- NEMA17.h / NEMA17.cpp
|     |- Kinematics3D.h / Kinematics3D.cpp
|     |- Gripper.h / Gripper.cpp
|- assets/
|- LICENSE
|- README.md
```

## Hardware

### Required

- Arduino Nano or Uno
- 3x NEMA17 stepper motors
- 3x step/dir drivers (A4988, DRV8825, or compatible)
- Shared enable line to all NEMA drivers
- 3x endstop switches (optional but recommended)
- 1x BYJ-48 stepper + ULN2003 board (gripper)
- Power supply sized for all motors and drivers

### Pinout

Defined in `firmware/RobotArm/Config_Pinout.h`.

| Function | Pin |
|---|---|
| Theta1 STEP | D6 |
| Theta1 DIR | D3 |
| Theta2 STEP | D5 |
| Theta2 DIR | D2 |
| Theta3 STEP | D7 |
| Theta3 DIR | D4 |
| Shared EN (NEMA drivers) | D8 |
| Theta1 endstop | D10 |
| Theta2 endstop | D9 |
| Theta3 endstop | D11 |
| Gripper IN1 | A0 |
| Gripper IN2 | A1 |
| Gripper IN3 | A2 |
| Gripper IN4 | A3 |

## Endstop Wiring and Logic

- Endstops are configured as `INPUT_PULLUP`.
- Default triggered logic is active-low (`ENDSTOP_ACTIVE_LOW true`).
- Practical wiring for default behavior:
	- one side of each switch to GND
	- other side to its endstop pin
- With this wiring:
	- switch open -> pin HIGH -> not triggered
	- switch pressed -> pin LOW -> triggered

If your hardware is active-high, change `ENDSTOP_ACTIVE_LOW` in `Config_Robot.h`.

## Configuration

Main config file: `firmware/RobotArm/Config_Robot.h`

Important fields:
- Serial:
	- `SERIAL_BAUD_RATE`
	- `SERIAL_TIMEOUT_MS`
- Motion:
	- `STEPS_PER_DEGREE`
	- `DEFAULT_MAX_SPEED`
	- `DEFAULT_ACCELERATION`
	- `DEFAULT_FEEDRATE`
- Inversions:
	- `MOTOR1_INVERT`, `MOTOR2_INVERT`, `MOTOR3_INVERT`
	- `GRIPPER_INVERT_DIRECTION`
- Home:
	- `HOME_THETA1`, `HOME_THETA2`, `HOME_THETA3`
- Endstop mode and homing:
	- `ENDSTOPS_INSTALLED`
	- `HOMING_DIR_THETA1/2/3`
	- `HOME_OFFSET_STEPS_THETA1/2/3`
	- `HOMING_SEEK_MAX_STEPS_THETA1/2/3`
	- `HOMING_RELEASE_MAX_STEPS`

## Homing Behavior

`G28` supports two compile-time modes:

### 1) Endstop Mode (`ENDSTOPS_INSTALLED true`)

Sequence:
1. release any axis that is already pressing a switch
2. seek all three endstops simultaneously
3. move by calibrated signed offsets (`HOME_OFFSET_STEPS_*`)
4. set logical pose to `HOME_THETA1/2/3`

Configured seek directions:
- Theta1 seeks positive
- Theta2 seeks positive
- Theta3 seeks negative

### 2) Software Home Mode (`ENDSTOPS_INSTALLED false`)

`G28` performs a regular move to `HOME_THETA1/2/3` without touching endstops.

## Build and Upload

### Arduino IDE

1. Open `firmware/RobotArm/RobotArm.ino`.
2. Select board: Arduino Nano (ATmega328P) or Uno.
3. Verify and upload.

### arduino-cli

Install core once:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
```

Compile:

```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/RobotArm
```

Upload (replace COM port):

```bash
arduino-cli upload -p COM3 --fqbn arduino:avr:nano firmware/RobotArm
```

## Serial Protocol

- Baud rate: `115200` (default)
- Line ending: newline or carriage return
- Startup message: `READY`
- Successful command: `ok`
- Error format: `Error: <message>`
- Emergency alarm line: `ALARM:ESTOP`

## Supported Commands

### Motion and Modes

| Command | Description |
|---|---|
| `G0` / `G1` | Move command (joint or Cartesian) |
| `G4 P<ms>` | Dwell (non-blocking state machine) |
| `G28 [F<feedrate>]` | Home arm |
| `G90` | Absolute Cartesian mode |
| `G91` | Relative Cartesian mode |

`G0/G1` parameter options:
- Joint space: `T1<deg>`, `T2<deg>`, `T3<deg>`
- Cartesian: `X`, `Y`, `Z` (mm)
- Feedrate: `F`

Parameter format note:
- No space between letter and value (example: `T110`, `T295`, `T3-5`, `X120`, `F40`).

### Motor Power and Status

| Command | Description |
|---|---|
| `M17` | Enable arm drivers |
| `M18` | Disable arm drivers |
| `M84` | Alias of `M18` |
| `M114` | Print Cartesian + joint state |
| `M119` | Print endstop states |
| `M400` | Wait until arm motion queue is done |
| `HELP` | Print compact command help |

### Gripper

| Command | Description |
|---|---|
| `M3` | Close gripper fully |
| `M3 S<mm>` | Move gripper to target position |
| `M5` | Open gripper |
| `M6` | Home gripper and zero position |
| `M3001` | Print gripper position |

### Emergency

| Command | Description |
|---|---|
| `M112` | Immediate emergency stop |
| `!` | Quick alias for `M112` |

After `M112`, motors are stopped and disabled. Re-enable with `M17` before new arm moves.

## Quick Test Sequence

```gcode
M17
M119
G28 F20
M114
G90
G1 T110 T295 T3-5 F40
M3 S10 F250
M5 F250
M400
```

Emergency during testing:

```text
M112
```

or

```text
!
```

## Calibration Guide

### 1) Verify directions

- If a NEMA axis moves opposite to expected, flip `MOTORx_INVERT`.
- If gripper close/open is reversed, set `GRIPPER_INVERT_DIRECTION`.

### 2) Verify steps per degree

`STEPS_PER_DEGREE` should include full-step count, microstep setting, and mechanical ratio.

General formula:

```text
steps_per_degree = (motor_steps_per_rev * microstepping * gear_ratio) / 360
```

### 3) Tune home offsets

`HOME_OFFSET_STEPS_THETA*` are signed step moves applied after endstop hit.

Tune workflow:
1. run `G28`
2. read `M114`
3. compare measured pose against desired `HOME_THETA*`
4. adjust offsets and reflash

## Safety Notes

- Firmware starts in manual-safe state (no automatic homing routine at boot).
- Keep feedrate conservative during first homing tests.
- Confirm endstop polarity and wiring before full-range moves.
- Maintain external hardware safety practices (power cutoff, clear workspace, supervised tests).

## Known Limitations

- Motion generation is simple coordinated stepping without advanced acceleration profiling.
- Some commands are blocking while waiting for completion (`G28`, `M6`, `M400` loops internally).
- G-code input line buffer is finite (`128` chars).
- Intended for low-cost AVR-class controllers; not a replacement for full CNC planners.

## Troubleshooting

### `Error: Homing failed`

- Check endstop wiring and polarity.
- Confirm `HOMING_DIR_THETA*` directions match mechanics.
- Increase `HOMING_SEEK_MAX_STEPS_*` if travel budget is too short.

### Axis moves wrong direction

- Toggle corresponding `MOTORx_INVERT`.

### Gripper moves opposite of command

- Set `GRIPPER_INVERT_DIRECTION true`.

### Arm does not move after emergency stop

- Send `M17` to re-enable motors, then command motion again.

## License

This project is licensed under the license in [LICENSE](LICENSE).
