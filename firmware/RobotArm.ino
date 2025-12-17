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
 * Author: Your Name
 * Date: 2024
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
    .maxSpeed = DEFAULT_MAX_SPEED,
    .acceleration = DEFAULT_ACCELERATION,
    .jerk = DEFAULT_JERK
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
    
    // Print startup banner
    printBanner();
    
    // Initialize robot arm
    Serial.println(F("Initializing robot arm..."));
    robotArm.begin(motor1Config, motor2Config, motor3Config, MOTORS_ENABLE_PIN);
    robotArm.setMotionParams(defaultMotion);
    Serial.println(F("  ✓ Robot arm initialized"));
    
    // Initialize gripper
    #if ENABLE_GRIPPER
    Serial.println(F("Initializing gripper..."));
    gripper.begin(gripperConfig);
    Serial.println(F("  ✓ Gripper initialized"));
    #endif
    
    // Initialize G-Code parser
    Serial.println(F("Initializing G-Code parser..."));
    #if ENABLE_GRIPPER
    gcode.begin(&robotArm, &gripper);
    #else
    gcode.begin(&robotArm, NULL);
    #endif
    gcode.setVerbose(GCODE_VERBOSE_MODE);
    Serial.println(F("  ✓ G-Code parser ready"));
    
    // Auto-home if enabled
    #if ENABLE_STARTUP_HOME
    Serial.println(F("\n⚠ WARNING: Open-loop homing - ensure robot"));
    Serial.println(F("   is at home position before powering on!"));
    Serial.println(F("\nMoving to home position..."));
    robotArm.enable();
    delay(MOTOR_ENABLE_DELAY_MS);
    robotArm.home(HOMING_FEEDRATE);
    while (robotArm.isMoving()) {
        robotArm.update();
        delay(1);
    }
    #if ENABLE_GRIPPER
    gripper.home(GRIPPER_HOMING_SPEED);
    while (gripper.isMoving()) {
        gripper.update();
        delay(1);
    }
    gripper.setZero();
    #endif
    Serial.println(F("  ✓ Homing complete"));
    #else
    Serial.println(F("\n⚠ IMPORTANT: Manually position robot at home"));
    Serial.println(F("   before sending movement commands!"));
    Serial.println(F("   Home = T1:0° T2:90° T3:0°"));
    #endif
    
    // Print ready message
    Serial.println(F("\n========================================"));
    Serial.println(F("System ready! Waiting for commands..."));
    Serial.println(F("========================================\n"));
    
    printQuickStart();
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
// HELPER FUNCTIONS
// ====================================================================

void printBanner() {
    Serial.println(F("\n========================================"));
    Serial.println(F("     3-DOF ROBOT ARM CONTROLLER"));
    Serial.println(F("========================================"));
    Serial.println(F("Author: NguyenLe15325"));
    Serial.println(F("Firmware Version: 1.0"));
    Serial.println(F("Board: Arduino Nano/Uno"));
    Serial.print(F("Baud Rate: "));
    Serial.println(SERIAL_BAUD_RATE);
    Serial.println(F("========================================\n"));
}

void printQuickStart() {
    Serial.println(F("========================================"));
    Serial.println(F("         QUICK START GUIDE"));
    Serial.println(F("========================================"));
    
    Serial.println(F("\n1. BEFORE POWER ON:"));
    Serial.println(F("   • Position robot: T1=0° T2=90° T3=0°"));
    Serial.println(F("   • Gripper: Fully OPEN (0mm)"));
    Serial.println(F("   • System assumes this position on startup"));
    
    Serial.println(F("\n2. VERIFY POSITION:"));
    Serial.println(F("   M17              ; Enable motors"));
    Serial.println(F("   M114             ; Should show T1:0 T2:90 T3:0"));
    Serial.println(F("   NOTE: G28 not needed - already at home!"));
    
    Serial.println(F("\n3. TEST DIRECTIONS (First time only):"));
    Serial.println(F("   G91              ; Relative mode"));
    Serial.println(F("   G0 X10 Y10 Z10 F10"));
    Serial.println(F("   Expected: X→Forward, Y→Up, Z→Right"));
    Serial.println(F("   If wrong → Set MOTOR_X_INVERT in Config"));
    Serial.println(F("   G90              ; Back to absolute mode"));
    
    Serial.println(F("\n4. BASIC COMMANDS:"));
    Serial.println(F("   G0 X200 Y50 Z100 ; Move Cartesian"));
    Serial.println(F("   G0 T10 T245 T30  ; Move joints (deg)"));
    Serial.println(F("   M3 / M5          ; Close/Open gripper"));
    Serial.println(F("   M114             ; Report position"));
    Serial.println(F("   HELP             ; Full reference"));
    
    Serial.println(F("\n========================================\n"));
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