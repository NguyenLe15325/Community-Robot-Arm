/*
 * ====================================================================
 *                    3-DOF ROBOT ARM CONTROLLER
 * ====================================================================
 * 
 * Features:
 * - 3-axis NEMA17 stepper control with coordinated motion
 * - BYJ-48 stepper gripper with ULN2003 driver
 * - G-Code command interface via serial
 * - Non-blocking motion and delays
 * - Inverse/Forward kinematics
 * 
 * Hardware:
 * - Arduino Nano/Uno
 * - 3x NEMA17 steppers with step/dir drivers
 * - 1x BYJ-48 stepper with ULN2003 driver
 * 
 * Author: NguyenLe15325
 * Date: 2025
 * ====================================================================
 */

// --- Include Configuration Files ---
#include "Config_Pinout.h"
#include "Config_Robot.h"

// --- Include Library Files ---
#include "Kinematics3D.h"
#include "NEMA17.h"
#include "Gripper.h"
#include "GCode.h"

// ====================================================================
// MOTOR CONFIGURATION
// ====================================================================

MotorConfig motor1Config = {
    .stepPin = MOTOR1_STEP_PIN,
    .dirPin = MOTOR1_DIR_PIN,
    .stepsPerDegree = STEPS_PER_DEGREE,
    .invertDirection = MOTOR1_INVERT
};

MotorConfig motor2Config = {
    .stepPin = MOTOR2_STEP_PIN,
    .dirPin = MOTOR2_DIR_PIN,
    .stepsPerDegree = STEPS_PER_DEGREE,
    .invertDirection = MOTOR2_INVERT
};

MotorConfig motor3Config = {
    .stepPin = MOTOR3_STEP_PIN,
    .dirPin = MOTOR3_DIR_PIN,
    .stepsPerDegree = STEPS_PER_DEGREE,
    .invertDirection = MOTOR3_INVERT
};

// ====================================================================
// MOTION CONFIGURATION
// ====================================================================

MotionParams defaultMotion = {
    .maxSpeed = DEFAULT_MAX_SPEED
};

// ====================================================================
// GRIPPER CONFIGURATION
// ====================================================================

GripperConfig gripperConfig = {
    .in1Pin = GRIPPER_IN1_PIN,
    .in2Pin = GRIPPER_IN2_PIN,
    .in3Pin = GRIPPER_IN3_PIN,
    .in4Pin = GRIPPER_IN4_PIN,
    .stepsPerMM = GRIPPER_STEPS_PER_MM,
    .minPosition = GRIPPER_MIN_POSITION,
    .maxPosition = GRIPPER_MAX_POSITION
};

// ====================================================================
// GLOBAL OBJECTS
// ====================================================================

NEMA17Controller robotArm;
BYJ48Gripper gripper;
GCodeParser gcode;

// ====================================================================
// SETUP
// ====================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < SERIAL_TIMEOUT_MS) {
        ; // Wait for serial port to connect
    }

    // Initialize robot arm
    robotArm.begin(motor1Config, motor2Config, motor3Config, MOTORS_ENABLE_PIN);
    robotArm.setMotionParams(defaultMotion);

    // Initialize gripper
    #if ENABLE_GRIPPER
    gripper.begin(gripperConfig);
    #endif

    // Initialize G-Code parser
    #if ENABLE_GRIPPER
    gcode.begin(&robotArm, &gripper);
    #else
    gcode.begin(&robotArm, NULL);
    #endif
    gcode.setVerbose(GCODE_VERBOSE_MODE);

    Serial.println(F("READY"));
}

// ====================================================================
// MAIN LOOP
// ====================================================================

void loop() {
    // Update robot arm motors
    robotArm.update();
    
    // Update gripper
    #if ENABLE_GRIPPER
    gripper.update();
    #endif
    
    // Process G-Code commands
    gcode.update();
}

// ====================================================================
// OPTIONAL: Custom Functions
// ====================================================================

// Add your custom functions here if needed
// Example: Emergency stop, custom sequences, sensor reading, etc.

/*
void emergencyStop() {
    robotArm.emergencyStop();
    gripper.stop();
    Serial.println(F("!!! EMERGENCY STOP !!!"));
}

void customPickPlace() {
    // Your custom pick and place routine
}
*/
