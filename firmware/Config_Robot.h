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
// - Microstepping mode: 2 (half-step)
// - Gear reduction ratio: 4.5:1
// Result: 5.0 steps/degree

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
#define DEFAULT_ACCELERATION    200.0       // Acceleration rate
#define DEFAULT_JERK            1000.0      // Jerk limit (future use)

// --- Feedrate Settings ---
#define DEFAULT_FEEDRATE        60.0        // Default feedrate (deg/s or mm/s)
#define HOMING_FEEDRATE         30.0        // Homing speed (slower for safety)

// --- Speed Presets (Optional) ---
#define SPEED_SLOW              45.0        // Conservative speed
#define SPEED_NORMAL            90.0        // Balanced speed
#define SPEED_FAST              120.0       // Aggressive speed


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

#define GRIPPER_STEPS_PER_MM    68.266667       // Adjust based on your mechanism

// --- Gripper Travel Limits ---
#define GRIPPER_MIN_POSITION    0.0         // Fully open (mm)
#define GRIPPER_MAX_POSITION    30.0        // Fully closed (mm)

// --- Gripper Speed ---
#define GRIPPER_DEFAULT_SPEED   300.0       // Default speed (steps/second)
#define GRIPPER_HOMING_SPEED    200.0       // Homing speed (slower)


// ====================================================================
// HOME POSITION
// ====================================================================

// Initial/home position for robot arm (in degrees)
#define HOME_THETA1             0.0         // Base: 0°
#define HOME_THETA2             90.0        // Shoulder: 90°
#define HOME_THETA3             0.0         // Elbow: 0°


// ====================================================================
// G-CODE PARSER SETTINGS
// ====================================================================

#define GCODE_VERBOSE_MODE      true        // Enable verbose output for debugging
#define GCODE_BUFFER_SIZE       128         // Maximum G-code line length


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

// --- Startup Behavior ---
// IMPORTANT: This is an open-loop system without limit switches!
// Auto-homing only moves motors to home angles - it does NOT verify
// the physical position. You MUST manually position the robot at 
// home before powering on, or position tracking will be incorrect.
//
// Recommended workflow:
// 1. Manually position robot at home (T1=0°, T2=90°, T3=0°)
// 2. Power on Arduino
// 3. Motors assume they're at home position
// 4. Send commands (G0, M114, etc.)
//
#define ENABLE_STARTUP_HOME     false       // Move to home on startup (usually not needed)

// --- Timing Parameters ---
#define MOTOR_ENABLE_DELAY_MS   100         // Delay after enabling motors
#define GRIPPER_SETTLE_TIME_MS  500         // Time for gripper to settle


// ====================================================================
// ADVANCED SETTINGS (Modify with caution)
// ====================================================================

// --- Step Pulse Width ---
#define STEP_PULSE_WIDTH_US     2           // Microseconds (min for most drivers)

// --- Motor Enable Logic ---
#define MOTOR_ENABLE_ACTIVE     LOW         // LOW = enabled (common for drivers)
#define MOTOR_DISABLE_ACTIVE    HIGH        // HIGH = disabled


#endif // CONFIG_ROBOT_H