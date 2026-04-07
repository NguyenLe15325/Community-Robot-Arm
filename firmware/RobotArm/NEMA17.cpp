#include "NEMA17.h"
#include "Config_Pinout.h"
#include "Config_Robot.h"
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

    endstopPins[0] = THETA1_ENDSTOP_PIN;
    endstopPins[1] = THETA2_ENDSTOP_PIN;
    endstopPins[2] = THETA3_ENDSTOP_PIN;

    #if ENDSTOPS_INSTALLED
    // Configure endstop inputs
    for (int i = 0; i < 3; i++) {
        pinMode(endstopPins[i], INPUT_PULLUP);
    }
    #endif
    
    // Initialize state
    moving = false;
    enabled = false;
    
    // Set default motion parameters
    motion.maxSpeed = 90.0;        // 90 deg/s
    
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
        moveStartSteps[i] = currentSteps[i];
    }

    leadAxisSteps = 0;
    leadAxisIndex = 0;
    moveRampPortion = MOVE_SMOOTHING_RAMP_PORTION_DEFAULT;
    moveMinSpeedScale = MOVE_SMOOTHING_MIN_SPEED_SCALE_DEFAULT;

    emergencyStopLatched = false;
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

void NEMA17Controller::setMotionSmoothing(float rampPortion, float minSpeedScale) {
    if (rampPortion < MOVE_SMOOTHING_RAMP_PORTION_MIN) {
        rampPortion = MOVE_SMOOTHING_RAMP_PORTION_MIN;
    } else if (rampPortion > MOVE_SMOOTHING_RAMP_PORTION_MAX) {
        rampPortion = MOVE_SMOOTHING_RAMP_PORTION_MAX;
    }

    if (minSpeedScale < MOVE_SMOOTHING_MIN_SPEED_SCALE_MIN) {
        minSpeedScale = MOVE_SMOOTHING_MIN_SPEED_SCALE_MIN;
    } else if (minSpeedScale > MOVE_SMOOTHING_MIN_SPEED_SCALE_MAX) {
        minSpeedScale = MOVE_SMOOTHING_MIN_SPEED_SCALE_MAX;
    }

    moveRampPortion = rampPortion;
    moveMinSpeedScale = minSpeedScale;
}

void NEMA17Controller::getMotionSmoothing(float& rampPortion, float& minSpeedScale) const {
    rampPortion = moveRampPortion;
    minSpeedScale = moveMinSpeedScale;
}

// --- Homing ---
bool NEMA17Controller::home(float feedrate) {
    if (!enabled) enable();

    emergencyStopLatched = false;

    #if !ENDSTOPS_INSTALLED
    // Fallback mode when no endstops are installed.
    JointAngles homeAngles;
    homeAngles.theta1 = HOME_THETA1 * (M_PI / 180.0);
    homeAngles.theta2 = HOME_THETA2 * (M_PI / 180.0);
    homeAngles.theta3 = HOME_THETA3 * (M_PI / 180.0);

    const float softwareHomeFeedrate = (feedrate > 0.0f) ? feedrate : HOMING_FEEDRATE;
    return moveToAngles(homeAngles, softwareHomeFeedrate);
    #else

    // Homing is executed in three phases:
    // 1) Release any already-pressed endstop (safe recovery)
    // 2) Simultaneous seek until each axis hits its endstop
    // 3) Simultaneous signed offset move to calibrated home
    const float seekFeedrate = (feedrate > 0.0f) ? feedrate : HOMING_SEEK_FEEDRATE;
    moving = false;

    if (!releasePressedEndstops(HOMING_RELEASE_FEEDRATE)) {
        return false;
    }

    if (!seekAllEndstops(seekFeedrate)) {
        return false;
    }

    if (!moveHomeOffsetSteps(HOMING_OFFSET_FEEDRATE)) {
        return false;
    }

    // After mechanical homing and calibrated offsets, define this as logical home.
    currentAngles.theta1 = HOME_THETA1 * (M_PI / 180.0);
    currentAngles.theta2 = HOME_THETA2 * (M_PI / 180.0);
    currentAngles.theta3 = HOME_THETA3 * (M_PI / 180.0);

    for (int i = 0; i < 3; i++) {
        currentSteps[i] = anglesToSteps(i, (i == 0) ? currentAngles.theta1 : (i == 1) ? currentAngles.theta2 : currentAngles.theta3);
        targetSteps[i] = currentSteps[i];
        lastStepTime[i] = micros();
    }

    moving = false;
    return true;
    #endif
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

    for (int i = 0; i < 3; i++) {
        moveStartSteps[i] = currentSteps[i];
    }
    
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

bool NEMA17Controller::getEndstopTriggered(uint8_t axisIndex) const {
    #if ENDSTOPS_INSTALLED
    if (axisIndex >= 3) {
        return false;
    }
    return isEndstopTriggered((int)axisIndex);
    #else
    (void)axisIndex;
    return false;
    #endif
}

// --- Emergency Stop ---
void NEMA17Controller::emergencyStop() {
    moving = false;
    for (int i = 0; i < 3; i++) {
        targetSteps[i] = currentSteps[i];
    }
}

bool NEMA17Controller::serviceEmergencyStopInput() {
    if (!pollEmergencyStopInput()) {
        return false;
    }

    emergencyStopLatched = true;
    emergencyStop();
    disable();
    return true;
}

bool NEMA17Controller::consumeEmergencyStopLatch() {
    bool wasLatched = emergencyStopLatched;
    emergencyStopLatched = false;
    return wasLatched;
}

// --- Update Loop (call this frequently) ---
void NEMA17Controller::update() {
    if (!moving || !enabled) return;
    
    unsigned long now = micros();
    bool allComplete = true;

    // Shared speed scale keeps axis ratios intact while easing start/stop.
    float speedScale = 1.0f;
    if (leadAxisSteps > 0) {
        long progressed = abs(currentSteps[leadAxisIndex] - moveStartSteps[leadAxisIndex]);
        float progress = (float)progressed / (float)leadAxisSteps;

        if (progress < 0.0f) progress = 0.0f;
        if (progress > 1.0f) progress = 1.0f;

        if (progress < moveRampPortion) {
            speedScale = progress / moveRampPortion;
        } else if (progress > (1.0f - moveRampPortion)) {
            speedScale = (1.0f - progress) / moveRampPortion;
        }

        if (speedScale < moveMinSpeedScale) {
            speedScale = moveMinSpeedScale;
        }
    }
    
    for (int i = 0; i < 3; i++) {
        if (currentSteps[i] != targetSteps[i]) {
            allComplete = false;

            unsigned long dynamicDelay = (unsigned long)(stepDelays[i] / speedScale);
            if (dynamicDelay < 50UL) {
                dynamicDelay = 50UL;
            }
            
            // Check if enough time has passed for next step
            if (now - lastStepTime[i] >= dynamicDelay) {
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
    leadAxisIndex = 0;
    
    for (int i = 0; i < 3; i++) {
        stepDiffs[i] = abs(targetSteps[i] - currentSteps[i]);
        if (stepDiffs[i] > maxSteps) {
            maxSteps = stepDiffs[i];
            leadAxisIndex = i;
        }
    }

    leadAxisSteps = maxSteps;
    
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
    stepMotorDirection(motorIndex, forward ? 1 : -1);
}

void NEMA17Controller::stepMotorDirection(int motorIndex, int8_t direction) {
    bool forward = (direction > 0);
    digitalWrite(motors[motorIndex].dirPin, forward ? HIGH : LOW);

    digitalWrite(motors[motorIndex].stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(motors[motorIndex].stepPin, LOW);

    currentSteps[motorIndex] += forward ? 1 : -1;
}

bool NEMA17Controller::readEndstopRaw(int motorIndex) const {
    const int activeLevel = ENDSTOP_ACTIVE_LOW ? LOW : HIGH;
    return digitalRead(endstopPins[motorIndex]) == activeLevel;
}

bool NEMA17Controller::isEndstopTriggered(int motorIndex) const {
    uint8_t hits = 0;
    uint8_t samples = (HOMING_SWITCH_DEBOUNCE_COUNT < 1) ? 1 : HOMING_SWITCH_DEBOUNCE_COUNT;

    for (uint8_t i = 0; i < samples; i++) {
        if (readEndstopRaw(motorIndex)) {
            hits++;
        }
        delayMicroseconds(50);
    }

    return hits == samples;
}

int8_t NEMA17Controller::thetaDirToStepDir(int motorIndex, int8_t thetaDirection) const {
    if (thetaDirection == 0) return 0;
    int8_t stepDir = (thetaDirection > 0) ? 1 : -1;
    if (motors[motorIndex].invertDirection) {
        stepDir = -stepDir;
    }
    return stepDir;
}

unsigned long NEMA17Controller::homingStepDelayUs(int motorIndex, float feedrate) const {
    float speed = (feedrate > 0.0f) ? feedrate : HOMING_SEEK_FEEDRATE;
    float stepsPerSec = speed * motors[motorIndex].stepsPerDegree;

    if (stepsPerSec < 1.0f) {
        stepsPerSec = 1.0f;
    }

    unsigned long delayUs = (unsigned long)(1000000.0f / stepsPerSec);
    if (delayUs < 50UL) {
        delayUs = 50UL;
    }
    return delayUs;
}

bool NEMA17Controller::pollEmergencyStopInput() {
    // Only consume dedicated emergency bytes so regular queued commands are preserved.
    while (Serial.available() > 0) {
        int nextValue = Serial.peek();
        if (nextValue < 0) {
            return false;
        }

        char c = (char)nextValue;

        if (c == '!') {
            Serial.read();
            return true;
        }

        if ((uint8_t)c == 0x18) {
            Serial.read();
            return true;
        }

        // Skip pending line endings to detect a following immediate emergency byte.
        if (c == '\r' || c == '\n') {
            Serial.read();
            continue;
        }

        break;
    }

    return false;
}

bool NEMA17Controller::releasePressedEndstops(float feedrate) {
    bool active[3] = {false, false, false};
    int8_t releaseDirections[3] = {0, 0, 0};
    unsigned long stepDelaysUs[3] = {0, 0, 0};
    unsigned long lastStepTimes[3] = {0, 0, 0};
    long releaseStepCount[3] = {0, 0, 0};

    const int8_t seekThetaDir[3] = {HOMING_DIR_THETA1, HOMING_DIR_THETA2, HOMING_DIR_THETA3};

    bool anyActive = false;
    for (int i = 0; i < 3; i++) {
        active[i] = isEndstopTriggered(i);
        if (active[i]) {
            anyActive = true;
            releaseDirections[i] = -thetaDirToStepDir(i, seekThetaDir[i]);
            stepDelaysUs[i] = homingStepDelayUs(i, feedrate);
            lastStepTimes[i] = micros();
        }
    }

    if (!anyActive) {
        return true;
    }

    while (true) {
        if (serviceEmergencyStopInput()) {
            return false;
        }

        bool allReleased = true;
        unsigned long now = micros();

        for (int i = 0; i < 3; i++) {
            if (!active[i]) {
                continue;
            }

            if (!isEndstopTriggered(i)) {
                active[i] = false;
                continue;
            }

            allReleased = false;

            if (releaseStepCount[i] >= HOMING_RELEASE_MAX_STEPS) {
                return false;
            }

            if (now - lastStepTimes[i] >= stepDelaysUs[i]) {
                stepMotorDirection(i, releaseDirections[i]);
                releaseStepCount[i]++;
                lastStepTimes[i] = now;
            }
        }

        if (allReleased) {
            return true;
        }
    }
}

bool NEMA17Controller::seekAllEndstops(float feedrate) {
    bool axisDone[3] = {false, false, false};
    uint8_t hitCounter[3] = {0, 0, 0};
    long seekStepCount[3] = {0, 0, 0};
    unsigned long stepDelaysUs[3] = {0, 0, 0};
    unsigned long lastStepTimes[3] = {0, 0, 0};

    const int8_t seekThetaDir[3] = {HOMING_DIR_THETA1, HOMING_DIR_THETA2, HOMING_DIR_THETA3};
    const int8_t seekStepDir[3] = {
        thetaDirToStepDir(0, seekThetaDir[0]),
        thetaDirToStepDir(1, seekThetaDir[1]),
        thetaDirToStepDir(2, seekThetaDir[2])
    };
    const long maxSeekSteps[3] = {
        HOMING_SEEK_MAX_STEPS_THETA1,
        HOMING_SEEK_MAX_STEPS_THETA2,
        HOMING_SEEK_MAX_STEPS_THETA3
    };

    for (int i = 0; i < 3; i++) {
        stepDelaysUs[i] = homingStepDelayUs(i, feedrate);
        lastStepTimes[i] = micros();
    }

    while (true) {
        if (serviceEmergencyStopInput()) {
            return false;
        }

        bool allDone = true;
        unsigned long now = micros();

        for (int i = 0; i < 3; i++) {
            if (axisDone[i]) {
                continue;
            }

            if (readEndstopRaw(i)) {
                if (hitCounter[i] < HOMING_SWITCH_DEBOUNCE_COUNT) {
                    hitCounter[i]++;
                }
                if (hitCounter[i] >= HOMING_SWITCH_DEBOUNCE_COUNT) {
                    axisDone[i] = true;
                    continue;
                }
            } else {
                hitCounter[i] = 0;
            }

            allDone = false;

            if (seekStepCount[i] >= maxSeekSteps[i]) {
                return false;
            }

            if (now - lastStepTimes[i] >= stepDelaysUs[i]) {
                stepMotorDirection(i, seekStepDir[i]);
                seekStepCount[i]++;
                lastStepTimes[i] = now;
            }
        }

        if (allDone) {
            return true;
        }
    }
}

bool NEMA17Controller::moveHomeOffsetSteps(float feedrate) {
    const long offsetStepsTheta[3] = {
        HOME_OFFSET_STEPS_THETA1,
        HOME_OFFSET_STEPS_THETA2,
        HOME_OFFSET_STEPS_THETA3
    };

    bool axisDone[3] = {false, false, false};
    long stepsRemaining[3] = {0, 0, 0};
    int8_t stepDirection[3] = {0, 0, 0};
    unsigned long stepDelaysUs[3] = {0, 0, 0};
    unsigned long lastStepTimes[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
        long offset = offsetStepsTheta[i];
        if (offset == 0) {
            axisDone[i] = true;
            continue;
        }

        int8_t thetaDir = (offset > 0) ? 1 : -1;
        stepDirection[i] = thetaDirToStepDir(i, thetaDir);
        stepsRemaining[i] = labs(offset);
        stepDelaysUs[i] = homingStepDelayUs(i, feedrate);
        lastStepTimes[i] = micros();
    }

    while (true) {
        if (serviceEmergencyStopInput()) {
            return false;
        }

        bool allDone = true;
        unsigned long now = micros();

        for (int i = 0; i < 3; i++) {
            if (axisDone[i]) {
                continue;
            }

            allDone = false;

            if (stepsRemaining[i] <= 0) {
                axisDone[i] = true;
                continue;
            }

            if (now - lastStepTimes[i] >= stepDelaysUs[i]) {
                stepMotorDirection(i, stepDirection[i]);
                stepsRemaining[i]--;
                lastStepTimes[i] = now;
            }
        }

        if (allDone) {
            return true;
        }
    }
}

bool NEMA17Controller::withinLimits(const JointAngles& angles) {
    if (angles.theta1 < THETA1_MIN_RAD || angles.theta1 > THETA1_MAX_RAD) return false;
    if (angles.theta2 < THETA2_MIN_RAD || angles.theta2 > THETA2_MAX_RAD) return false;
    if (angles.theta3 < THETA3_MIN_RAD || angles.theta3 > THETA3_MAX_RAD) return false;
    return true;
}