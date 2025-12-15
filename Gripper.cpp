#include "Gripper.h"

// Half-step sequence for ULN2003 driver (8 steps per cycle)
// BYJ-48: 4096 steps per revolution in half-step mode
const uint8_t BYJ48Gripper::STEP_SEQUENCE[8][4] = {
    {1, 0, 0, 0},  // Step 0
    {1, 1, 0, 0},  // Step 1
    {0, 1, 0, 0},  // Step 2
    {0, 1, 1, 0},  // Step 3
    {0, 0, 1, 0},  // Step 4
    {0, 0, 1, 1},  // Step 5
    {0, 0, 0, 1},  // Step 6
    {1, 0, 0, 1}   // Step 7
};

// --- Initialization ---
void BYJ48Gripper::begin(const GripperConfig& cfg) {
    config = cfg;
    
    // Configure pins
    pinMode(config.in1Pin, OUTPUT);
    pinMode(config.in2Pin, OUTPUT);
    pinMode(config.in3Pin, OUTPUT);
    pinMode(config.in4Pin, OUTPUT);
    
    // Initialize state
    currentSteps = 0;
    currentPosition = 0;
    targetSteps = 0;
    moving = false;
    enabled = false;
    currentStep = 0;
    direction = 0;
    lastStepTime = 0;
    stepDelay = 2000; // Default: 500 steps/sec
    
    // Disable motor initially
    disable();
}

// --- Movement Functions ---
bool BYJ48Gripper::moveToPosition(float position, float speed) {
    // Clamp position to limits
    if (position < config.minPosition) position = config.minPosition;
    if (position > config.maxPosition) position = config.maxPosition;
    
    targetSteps = positionToSteps(position);
    
    // Calculate step delay from speed
    if (speed < 1.0) speed = 1.0;
    if (speed > 500.0) speed = 500.0;
    stepDelay = (unsigned long)(1000000.0 / speed);
    
    // Determine direction
    if (targetSteps > currentSteps) {
        direction = 1;  // Closing
    } else if (targetSteps < currentSteps) {
        direction = -1; // Opening
    } else {
        direction = 0;
        moving = false;
        return true;
    }
    
    enable();
    moving = true;
    return true;
}

bool BYJ48Gripper::moveRelative(float distance, float speed) {
    float targetPos = currentPosition + distance;
    return moveToPosition(targetPos, speed);
}

void BYJ48Gripper::open(float speed) {
    moveToPosition(config.minPosition, speed);
}

void BYJ48Gripper::close(float speed) {
    moveToPosition(config.maxPosition, speed);
}

void BYJ48Gripper::stop() {
    moving = false;
    targetSteps = currentSteps;
    direction = 0;
}

void BYJ48Gripper::enable() {
    enabled = true;
    setMotorPins(currentStep);
}

void BYJ48Gripper::disable() {
    enabled = false;
    // Turn off all coils
    digitalWrite(config.in1Pin, LOW);
    digitalWrite(config.in2Pin, LOW);
    digitalWrite(config.in3Pin, LOW);
    digitalWrite(config.in4Pin, LOW);
}

void BYJ48Gripper::home(float speed) {
    // Home by opening fully
    // Assumes mechanical stop at fully open position
    open(speed);
}

// --- Update Loop ---
void BYJ48Gripper::update() {
    if (!moving || !enabled) return;
    
    unsigned long now = micros();
    
    // Check if it's time for the next step
    if (now - lastStepTime >= stepDelay) {
        if (currentSteps != targetSteps) {
            doStep();
            lastStepTime = now;
        } else {
            moving = false;
            direction = 0;
        }
    }
}

// --- Private Helper Functions ---
void BYJ48Gripper::doStep() {
    // Update step sequence position
    if (direction > 0) {
        currentStep = (currentStep + 1) % 8;
        currentSteps++;
    } else if (direction < 0) {
        currentStep = (currentStep == 0) ? 7 : currentStep - 1;
        currentSteps--;
    }
    
    // Update motor pins
    setMotorPins(currentStep);
    
    // Update position
    currentPosition = stepsToPosition(currentSteps);
}

void BYJ48Gripper::setMotorPins(uint8_t step) {
    if (!enabled) return;
    
    digitalWrite(config.in1Pin, STEP_SEQUENCE[step][0]);
    digitalWrite(config.in2Pin, STEP_SEQUENCE[step][1]);
    digitalWrite(config.in3Pin, STEP_SEQUENCE[step][2]);
    digitalWrite(config.in4Pin, STEP_SEQUENCE[step][3]);
}

long BYJ48Gripper::positionToSteps(float position) {
    return (long)(position * config.stepsPerMM);
}

float BYJ48Gripper::stepsToPosition(long steps) {
    return (float)steps / config.stepsPerMM;
}