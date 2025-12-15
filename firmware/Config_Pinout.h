#ifndef CONFIG_PINOUT_H
#define CONFIG_PINOUT_H

/*
 * ====================================================================
 *                    ROBOT ARM PIN CONFIGURATION
 * ====================================================================
 * 
 * Configure all hardware pins here for easy modification.
 * Adjust pin assignments based on your Arduino board and wiring.
 * 
 * Board: Arduino Nano / Uno
 * ====================================================================
 */

// ====================================================================
// STEPPER MOTORS (NEMA17 with Step/Dir drivers)
// ====================================================================

// --- Motor 1: Base Rotation (Theta1) ---
#define MOTOR1_STEP_PIN         6
#define MOTOR1_DIR_PIN          3

// --- Motor 2: Shoulder (Theta2) ---
#define MOTOR2_STEP_PIN         5
#define MOTOR2_DIR_PIN          2

// --- Motor 3: Elbow (Theta3) ---
#define MOTOR3_STEP_PIN         7
#define MOTOR3_DIR_PIN          4

// --- Shared Enable Pin for All NEMA17 Motors ---
#define MOTORS_ENABLE_PIN       8


// ====================================================================
// GRIPPER (BYJ-48 with ULN2003 Driver)
// ====================================================================

#define GRIPPER_IN1_PIN         A0
#define GRIPPER_IN2_PIN         A1
#define GRIPPER_IN3_PIN         A2
#define GRIPPER_IN4_PIN         A3

// ====================================================================

#endif // CONFIG_PINOUT_H