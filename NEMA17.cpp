#include "NEMA17.h"
#include <math.h>

// --- Initialization ---
void NEMA17Controller::begin(const MotorConfig& motor1, const MotorConfig& motor2, const MotorConfig& motor3, uint8_t enablePin) {
    motors[0] = motor1;
    motors[1] = motor2;
    motors[2] = motor3;
    sharedEnablePin = enablePin;
    
    // Configure pins
    for (int i = 0; i < 3; i++) {
        pinMode(motors[i].stepPin, OUTPUT);
        pinMode(motors[i].dirPin, OUTPUT);
        digitalWrite(motors[i].stepPin, LOW);
        digitalWrite(motors[i].dirPin, LOW);
    }
    
    // Configure shared enable pin
    pinMode(sharedEnablePin, OUTPUT);
    digitalWrite(sharedEnablePin, HIGH); // Disabled initially
    
    // Initialize state
    moving = false;
    enabled = false;
    
    // Set default motion parameters
    motion.maxSpeed = 90.0;        // 90 deg/s
    motion.acceleration = 200.0;   // 200 deg/s²
    motion.jerk = 1000.0;
    
    // Initialize to home position: theta1=0, theta2=90, theta3=0
    currentAngles.theta1 = 0.0;
    currentAngles.theta2 = 90.0 * (M_PI / 180.0);
    currentAngles.theta3 = 0.0;
    
    // Initialize steps to match home angles
    currentSteps[0] = anglesToSteps(0, currentAngles.theta1);  // 0 steps
    currentSteps[1] = anglesToSteps(1, currentAngles.theta2);  // 450 steps (90° × 5)
    currentSteps[2] = anglesToSteps(2, currentAngles.theta3);  // 0 steps
    
    for (int i = 0; i < 3; i++) {
        targetSteps[i] = currentSteps[i];  // Target = current (no movement)
        lastStepTime[i] = 0;
    }
}

void NEMA17Controller::enable() {
    digitalWrite(sharedEnablePin, LOW); // Active LOW for most drivers
    enabled = true;
}

void NEMA17Controller::disable() {
    digitalWrite(sharedEnablePin, HIGH);
    enabled = false;
}

void NEMA17Controller::setMotionParams(const MotionParams& params) {
    motion = params;
}

// --- Homing ---
bool NEMA17Controller::home(float feedrate) {
    if (!enabled) enable();
    
    // Home position: theta1=0°, theta2=90°, theta3=0°
    JointAngles homeAngles;
    homeAngles.theta1 = 0.0;
    homeAngles.theta2 = 90.0 * (M_PI / 180.0);
    homeAngles.theta3 = 0.0;
    
    return moveToAngles(homeAngles, feedrate);
}

// --- Movement Functions ---
bool NEMA17Controller::moveToAngles(const JointAngles& target, float feedrate) {
    if (!enabled) {
        return false;
    }
    
    // Check limits
    if (!withinLimits(target)) {
        return false;
    }
    
    // Calculate target steps
    targetSteps[0] = anglesToSteps(0, target.theta1);
    targetSteps[1] = anglesToSteps(1, target.theta2);
    targetSteps[2] = anglesToSteps(2, target.theta3);
    
    // Calculate coordinated motion profile
    calculateCoordinatedMotion(target, feedrate);
    
    currentAngles = target;
    moving = true;
    
    return true;
}

bool NEMA17Controller::moveToPosition(const CartesianPos& target, float feedrate) {
    JointAngles targetAngles;
    
    // Compute inverse kinematics
    if (!kinematics.inverseKinematics(target, targetAngles)) {
        return false; // Target unreachable
    }
    
    return moveToAngles(targetAngles, feedrate);
}

// --- State Queries ---
JointAngles NEMA17Controller::getCurrentAngles() const {
    return currentAngles;
}

CartesianPos NEMA17Controller::getCurrentPosition() const {
    return kinematics.forwardKinematics(currentAngles);
}

// --- Emergency Stop ---
void NEMA17Controller::emergencyStop() {
    moving = false;
    for (int i = 0; i < 3; i++) {
        targetSteps[i] = currentSteps[i];
    }
}

// --- Update Loop (call this frequently) ---
void NEMA17Controller::update() {
    if (!moving || !enabled) return;
    
    unsigned long now = micros();
    bool allComplete = true;
    
    for (int i = 0; i < 3; i++) {
        if (currentSteps[i] != targetSteps[i]) {
            allComplete = false;
            
            // Check if enough time has passed for next step
            if (now - lastStepTime[i] >= stepDelays[i]) {
                stepMotor(i);
                lastStepTime[i] = now;
            }
        }
    }
    
    if (allComplete) {
        moving = false;
    }
}

// --- Private Helper Functions ---
long NEMA17Controller::anglesToSteps(int motorIndex, float angleRad) {
    float angleDeg = angleRad * (180.0 / M_PI);
    long steps = (long)(angleDeg * motors[motorIndex].stepsPerDegree);
    return motors[motorIndex].invertDirection ? -steps : steps;
}

float NEMA17Controller::stepsToAngle(int motorIndex, long steps) {
    if (motors[motorIndex].invertDirection) steps = -steps;
    float angleDeg = (float)steps / motors[motorIndex].stepsPerDegree;
    return angleDeg * (M_PI / 180.0);
}

void NEMA17Controller::calculateCoordinatedMotion(const JointAngles& target, float feedrate) {
    // Calculate step differences
    long stepDiffs[3];
    long maxSteps = 0;
    
    for (int i = 0; i < 3; i++) {
        stepDiffs[i] = abs(targetSteps[i] - currentSteps[i]);
        if (stepDiffs[i] > maxSteps) {
            maxSteps = stepDiffs[i];
        }
    }
    
    if (maxSteps == 0) {
        moving = false;
        return;
    }
    
    // Use provided feedrate or default
    float speed = (feedrate > 0) ? feedrate : motion.maxSpeed;
    
    // Calculate timing for coordinated motion
    // The motor with the most steps determines the overall speed
    for (int i = 0; i < 3; i++) {
        if (stepDiffs[i] == 0) {
            stepDelays[i] = 1000000; // Very long delay (won't move)
        } else {
            // Calculate proportional speed for this motor
            float ratio = (float)stepDiffs[i] / (float)maxSteps;
            float motorSpeed = speed * ratio; // deg/s
            float stepsPerSec = motorSpeed * motors[i].stepsPerDegree;
            stepDelays[i] = (unsigned long)(1000000.0 / stepsPerSec);
            
            // Safety limit: minimum 50us between steps
            if (stepDelays[i] < 50) stepDelays[i] = 50;
        }
    }
}

void NEMA17Controller::stepMotor(int motorIndex) {
    // Determine direction
    bool forward = targetSteps[motorIndex] > currentSteps[motorIndex];
    digitalWrite(motors[motorIndex].dirPin, forward ? HIGH : LOW);
    
    // Pulse the step pin
    digitalWrite(motors[motorIndex].stepPin, HIGH);
    delayMicroseconds(2); // Minimum pulse width for most drivers
    digitalWrite(motors[motorIndex].stepPin, LOW);
    
    // Update position
    currentSteps[motorIndex] += forward ? 1 : -1;
}

bool NEMA17Controller::withinLimits(const JointAngles& angles) {
    if (angles.theta1 < THETA1_MIN_RAD || angles.theta1 > THETA1_MAX_RAD) return false;
    if (angles.theta2 < THETA2_MIN_RAD || angles.theta2 > THETA2_MAX_RAD) return false;
    if (angles.theta3 < THETA3_MIN_RAD || angles.theta3 > THETA3_MAX_RAD) return false;
    return true;
}