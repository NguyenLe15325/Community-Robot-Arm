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
    Serial.println(F("Firmware Version: 1.0"));
    Serial.println(F("Board: Arduino Nano/Uno"));
    Serial.print(F("Baud Rate: "));
    Serial.println(SERIAL_BAUD_RATE);
    Serial.println(F("========================================\n"));
}

void printQuickStart() {
    Serial.println(F("========================================"));
    Serial.println(F("         COMMAND REFERENCE"));
    Serial.println(F("========================================"));
    
    Serial.println(F("\n--- MOTION COMMANDS ---"));
    Serial.println(F("G0/G1 X<pos> Y<pos> Z<pos> F<speed>"));
    Serial.println(F("  Move to Cartesian position (mm)"));
    Serial.println(F("  Example: G0 X200 Y50 Z100 F60"));
    Serial.println();
    Serial.println(F("G0/G1 T1<deg> T2<deg> T3<deg> F<speed>"));
    Serial.println(F("  Move to joint angles (degrees)"));
    Serial.println(F("  Example: G0 T10 T245 T30 F90"));
    Serial.println();
    Serial.println(F("G4 P<ms>"));
    Serial.println(F("  Non-blocking delay in milliseconds"));
    Serial.println(F("  Example: G4 P500 (wait 0.5 seconds)"));
    Serial.println();
    Serial.println(F("G28"));
    Serial.println(F("  Home robot to initial position"));
    Serial.println(F("  (Theta1=0, Theta2=90, Theta3=0)"));
    Serial.println();
    Serial.println(F("G90"));
    Serial.println(F("  Absolute positioning mode (default)"));
    Serial.println();
    Serial.println(F("G91"));
    Serial.println(F("  Relative positioning mode"));
    
    #if ENABLE_GRIPPER
    Serial.println(F("\n--- GRIPPER COMMANDS ---"));
    Serial.println(F("M3"));
    Serial.println(F("  Close gripper fully"));
    Serial.println(F("  Example: M3 F400 (with custom speed)"));
    Serial.println();
    Serial.println(F("M3 S<pos>"));
    Serial.println(F("  Move gripper to position (0-30mm)"));
    Serial.println(F("  Example: M3 S15 (half closed)"));
    Serial.println();
    Serial.println(F("M5"));
    Serial.println(F("  Open gripper fully"));
    Serial.println();
    Serial.println(F("M6"));
    Serial.println(F("  Home gripper (open & set zero)"));
    #endif
    
    Serial.println(F("\n--- SYSTEM COMMANDS ---"));
    Serial.println(F("M17"));
    Serial.println(F("  Enable all motors"));
    Serial.println();
    Serial.println(F("M18 / M84"));
    Serial.println(F("  Disable all motors"));
    Serial.println();
    Serial.println(F("M114"));
    Serial.println(F("  Report current position"));
    Serial.println(F("  (Cartesian + joint angles + gripper)"));
    Serial.println();
    Serial.println(F("M400"));
    Serial.println(F("  Wait for all moves to finish"));
    
    Serial.println(F("\n--- JOINT LIMITS ---"));
    Serial.println(F("  Theta1 (Base):     -90° to +90°"));
    Serial.println(F("  Theta2 (Shoulder):   0° to 130°"));
    Serial.println(F("  Theta3 (Elbow):    -17° to +90°"));
    
    Serial.println(F("\n--- EXAMPLE SEQUENCES ---"));
    Serial.println(F("\n1. Basic Startup:"));
    Serial.println(F("   M17              ; Enable motors"));
    Serial.println(F("   G28              ; Home robot"));
    #if ENABLE_GRIPPER
    Serial.println(F("   M6               ; Home gripper"));
    #endif
    
    Serial.println(F("\n2. Simple Movement:"));
    Serial.println(F("   G0 X200 Y50 Z100 F60"));
    Serial.println(F("   M400             ; Wait"));
    Serial.println(F("   G0 T10 T245 T30  ; Joint mode"));
    
    #if ENABLE_GRIPPER
    Serial.println(F("\n3. Pick and Place:"));
    Serial.println(F("   G0 X200 Y100 Z50 ; Move to pick"));
    Serial.println(F("   M400             ; Wait for motion"));
    Serial.println(F("   G4 P200          ; Stabilize"));
    Serial.println(F("   M5               ; Open gripper"));
    Serial.println(F("   G4 P500          ; Wait for open"));
    Serial.println(F("   M3               ; Close gripper"));
    Serial.println(F("   G4 P800          ; Wait for grip"));
    Serial.println(F("   G0 X150 Y-50 Z80 ; Move to place"));
    Serial.println(F("   M400"));
    Serial.println(F("   M5               ; Release"));
    Serial.println(F("   G4 P500"));
    Serial.println(F("   G28              ; Return home"));
    
    Serial.println(F("\n4. Gripper Test:"));
    Serial.println(F("   M6               ; Home gripper"));
    Serial.println(F("   G4 P500"));
    Serial.println(F("   M3 S5            ; Close to 5mm"));
    Serial.println(F("   G4 P1000"));
    Serial.println(F("   M3 S15           ; Close to 15mm"));
    Serial.println(F("   G4 P1000"));
    Serial.println(F("   M3               ; Fully close"));
    Serial.println(F("   G4 P1000"));
    Serial.println(F("   M5               ; Open"));
    #endif
    
    Serial.println(F("\n--- PARAMETERS ---"));
    Serial.println(F("  F<value>  Feedrate (deg/s or mm/s)"));
    Serial.println(F("  S<value>  Gripper position (mm)"));
    Serial.println(F("  P<value>  Delay time (ms)"));
    Serial.println(F("  X Y Z     Cartesian coordinates (mm)"));
    Serial.println(F("  T1 T2 T3  Joint angles (degrees)"));
    
    Serial.println(F("\n--- TIPS ---"));
    Serial.println(F("  • Always home (G28/M6) after power on"));
    Serial.println(F("  • Use M400 before G4 for accurate delays"));
    Serial.println(F("  • Add G4 P500 after gripper commands"));
    Serial.println(F("  • Check position with M114 anytime"));
    Serial.println(F("  • Disable motors (M18) when not in use"));
    Serial.println(F("  • All commands return 'ok' when done"));
    Serial.println(F("  • Use ';' for comments in G-code"));
    
    Serial.println(F("\n--- TROUBLESHOOTING ---"));
    Serial.println(F("  Motors not moving?"));
    Serial.println(F("    → Send M17 to enable motors"));
    Serial.println(F("  Position inaccurate?"));
    Serial.println(F("    → Home with G28 and M6"));
    Serial.println(F("  Movement fails?"));
    Serial.println(F("    → Check joint limits with M114"));
    Serial.println(F("  Gripper wrong direction?"));
    Serial.println(F("    → Swap two wires on ULN2003"));
    
    Serial.println(F("\n========================================"));
    Serial.println(F("Ready for commands! Type help anytime."));
    Serial.println(F("========================================\n"));
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