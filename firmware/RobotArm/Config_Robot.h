#ifndef CONFIG_ROBOT_H
#define CONFIG_ROBOT_H

/*
 * ====================================================================
 *                    ROBOT ARM CONFIGURATION
 * ====================================================================
 * 
 * Configure all robot parameters here:
 * - Motor specifications
 * - Gripper settings
 * - Motion parameters
 * - Serial communication
 * 
 * Adjust these values to match your hardware setup.
 * ====================================================================
 */

// ====================================================================
// SERIAL COMMUNICATION
// ====================================================================

#define SERIAL_BAUD_RATE        115200      // Serial communication speed
#define SERIAL_TIMEOUT_MS       3000        // Timeout for serial init


// ====================================================================
// NEMA17 STEPPER MOTOR CONFIGURATION
// ====================================================================

// --- Steps Calculation ---
// Formula: (steps_per_rev × microstepping × gear_ratio) ÷ 360
// Example: (200 × 2 × 4.5) ÷ 360 = 5.0 steps/degree
//
// Your setup:
// - Steps per revolution: 200 (standard NEMA17)
// - Microstepping mode: 8
// - Gear reduction ratio: 4.5:1
// Result: 20.0 steps/degree

#define STEPS_PER_DEGREE        20.0         // Calculated steps per degree

// --- Motor Direction ---
// Set to true if motor rotates backwards
// (Easier to flip this than rewire the motor)

#define MOTOR1_INVERT           false       // Base rotation
#define MOTOR2_INVERT           false       // Shoulder
#define MOTOR3_INVERT           false       // Elbow


// ====================================================================
// MOTION PARAMETERS
// ====================================================================

// --- Speed Settings (degrees/second) ---
#define DEFAULT_MAX_SPEED       90.0        // Maximum rotational speed

// --- Feedrate Settings ---
#define DEFAULT_FEEDRATE        60.0        // Default feedrate (joint-space deg/s)
#define HOMING_FEEDRATE         40.0        // Homing speed (slower for safety)

// --- Motion Smoothing Profile (simple eased trapezoid) ---
// Ramp portion is the accel/decel zone fraction at each move edge.
// Min speed scale is the floor ratio relative to cruise speed.
#define MOVE_SMOOTHING_RAMP_PORTION_DEFAULT      0.20f
#define MOVE_SMOOTHING_MIN_SPEED_SCALE_DEFAULT   0.35f

// Runtime clamp limits for M205 tuning.
#define MOVE_SMOOTHING_RAMP_PORTION_MIN          0.05f
#define MOVE_SMOOTHING_RAMP_PORTION_MAX          0.45f
#define MOVE_SMOOTHING_MIN_SPEED_SCALE_MIN       0.10f
#define MOVE_SMOOTHING_MIN_SPEED_SCALE_MAX       1.00f


// ====================================================================
// GRIPPER CONFIGURATION (BYJ-48)
// ====================================================================

// --- Gripper Mechanics ---
// BYJ-48 stepper: 4096 steps per revolution (half-step mode)
// 
// Calculate stepsPerMM:
// - Measure: How many mm does gripper move in 1 full rotation?
// - Formula: 4096 ÷ mm_per_rotation = stepsPerMM
// 
// Examples:
// - 10mm per rotation → 4096 ÷ 10 = 409.6 steps/mm
// - 5mm per rotation  → 4096 ÷ 5  = 819.2 steps/mm
// - 20mm per rotation → 4096 ÷ 20 = 204.8 steps/mm

#define GRIPPER_STEPS_PER_MM    68.266667       // Adjust based on your mechanism: 30mm/2048steps

// --- Gripper Travel Limits ---
#define GRIPPER_MIN_POSITION    0.0         // Fully open (mm)
#define GRIPPER_MAX_POSITION    30.0        // Fully closed (mm)

// --- Gripper Speed ---
#define GRIPPER_DEFAULT_SPEED   15         // Default speed (mm/s)
#define GRIPPER_HOMING_SPEED    10         // Homing speed (mm/s)

// --- Gripper Direction ---
// Set true to reverse gripper motion direction in firmware (no rewiring required).
#define GRIPPER_INVERT_DIRECTION true

// --- GRIPPER SPEED NOTES ---
// • G-code F for M3/M5/M6 is interpreted as mm/s.
// • Firmware converts mm/s to steps/s using GRIPPER_STEPS_PER_MM.
// • Internal stepper limits are 1-500 steps/second (enforced in Gripper.cpp).
// • Effective mm/s range is approximately: [1/GRIPPER_STEPS_PER_MM, 500/GRIPPER_STEPS_PER_MM].


// ====================================================================
// HOME POSITION
// ====================================================================

// Initial/home position for robot arm (in degrees)
#define HOME_THETA1             0.0         // Base: 0°
#define HOME_THETA2             90.0        // Shoulder: 90°
#define HOME_THETA3             0.0         // Elbow: 0°

// ====================================================================
// ENDSTOP HOMING CONFIGURATION
// ====================================================================

// Set false when no endstops are installed.
// - true: G28 uses endstop seek + calibrated offset
// - false: G28 falls back to software home move (HOME_THETA1/2/3)
#define ENDSTOPS_INSTALLED             true

// Endstops are read with INPUT_PULLUP by default.
// Triggered state = LOW means switch pulled to GND.
#define ENDSTOP_ACTIVE_LOW              true

// Homing seek direction in THETA space:
// +1 = increase theta, -1 = decrease theta
// Requested behavior:
// - Theta1 and Theta2 seek in positive direction
// - Theta3 seeks in negative direction
#define HOMING_DIR_THETA1               1
#define HOMING_DIR_THETA2               1
#define HOMING_DIR_THETA3              -1

// Signed calibrated offset after endstop hit, in motor steps.
// Positive offset = theta increases, negative offset = theta decreases.
// angle_to_step = STEPS_PER_DEGREE (already includes microstep mode and belt reduction ratio).
// Initial estimate below; fine-tune each axis after first homing tests.
#define HOME_OFFSET_STEPS_THETA1        -1800L // -((theta1_max - theta1_home) * angle_to_step) = -((90 - 0) * 20) = -1800 // Approx
#define HOME_OFFSET_STEPS_THETA2         -870L // -((theta2_max - theta2_home) * angle_to_step) = -((130 - 90) * 20) = -800 // Approx
#define HOME_OFFSET_STEPS_THETA3          300L // +((theta3_home - theta3_min) * angle_to_step) = +((0 - (-17)) * 20) = +340 // Approx

// Homing safety limits
#define HOMING_SWITCH_DEBOUNCE_COUNT    3    // Consecutive reads required
#define HOMING_RELEASE_MAX_STEPS        400L // Release budget from already-pressed switch: 400/20 = 20deg
#define HOMING_SEEK_MAX_STEPS_THETA1   4200L // ((theta1_max - theta1_min) * angle_to_step) + margin = ((90 - (-90)) * 20) + 600 = 4200
#define HOMING_SEEK_MAX_STEPS_THETA2   3200L // ((theta2_max - theta2_min) * angle_to_step) + margin = ((130 - 0) * 20) + 600 = 3200
#define HOMING_SEEK_MAX_STEPS_THETA3   2800L // ((theta3_max - theta3_min) * angle_to_step) + margin = ((90 - (-17)) * 20) + 660 = 2800

// Homing speeds (deg/s)
#define HOMING_SEEK_FEEDRATE            HOMING_FEEDRATE
#define HOMING_RELEASE_FEEDRATE         30.0
#define HOMING_OFFSET_FEEDRATE          30.0


// ====================================================================
// G-CODE PARSER SETTINGS
// ====================================================================

#define GCODE_VERBOSE_MODE      true        // Enable verbose output for debugging

// ====================================================================
// --- Information Only (hardcoded in firmware) ---
// GCODE_BUFFER_SIZE: 128 characters max per line
// STEP_PULSE_WIDTH: 2 microseconds
// GRIPPER_SETTLE_TIME: Add G4 P<ms> manually in G-code

// ====================================================================
// SAFETY LIMITS (From Kinematics3D.h)
// ====================================================================

// Joint angle limits are defined in Kinematics3D.h:
// - THETA1: -90° to +90°
// - THETA2: 0° to 130°
// - THETA3: -17° to +90°
//
// Workspace limits are calculated automatically in Kinematics3D.h


// ====================================================================
// OPTIONAL FEATURES
// ====================================================================

// --- Enable/Disable Features ---
#define ENABLE_GRIPPER          true        // Enable gripper control



#endif // CONFIG_ROBOT_H