#ifndef NEMA17_H
#define NEMA17_H

#include <Arduino.h>
#include "Kinematics3D.h"

// --- Motor Configuration ---
struct MotorConfig {
    uint8_t stepPin;
    uint8_t dirPin;
    float stepsPerDegree;  // Steps per degree (accounting for microstepping & gearing)
    bool invertDirection;   // Invert direction if motor is mounted backwards
};

// --- Motion Parameters ---
struct MotionParams {
    float maxSpeed;         // Maximum speed in degrees/second
};

class NEMA17Controller {
public:
    /**
     * @brief Initialize the stepper controller
     * @param motor1 Configuration for theta1 motor
     * @param motor2 Configuration for theta2 motor
     * @param motor3 Configuration for theta3 motor
     * @param enablePin Shared enable pin for all motors
     */
    void begin(const MotorConfig& motor1, const MotorConfig& motor2, const MotorConfig& motor3, uint8_t enablePin);
    
    /**
     * @brief Enable all motors
     */
    void enable();

    /**
     * @brief Query whether the motor drivers are currently enabled.
     * @return true when enabled (drivers powered), false when disabled.
     */
    bool isEnabled() const { return enabled; }
    
    /**
     * @brief Disable all motors (reduces power consumption & heat)
     */
    void disable();
    
    /**
     * @brief Set motion parameters for coordinated movement
     */
    void setMotionParams(const MotionParams& params);

    /**
     * @brief Configure simple easing profile for move start/end.
     * @param rampPortion Fraction of move used for accel/decel zones.
     * @param minSpeedScale Minimum speed scale at the move edges.
     */
    void setMotionSmoothing(float rampPortion, float minSpeedScale);

    /**
     * @brief Read active easing profile values.
     */
    void getMotionSmoothing(float& rampPortion, float& minSpeedScale) const;
    
    /**
     * @brief Home robot
     * - ENDSTOPS_INSTALLED=true: endstop seek + calibrated offset
     * - ENDSTOPS_INSTALLED=false: software move to HOME_THETA1/2/3
     * @param feedrate Homing speed in degrees/second (0 = use configured default)
     * @return true if homing successful
     */
    bool home(float feedrate = 30.0f);
    
    /**
     * @brief Move to target joint angles with coordinated motion
     * @param target Target joint angles in radians
     * @param feedrate Movement speed (0 = use default maxSpeed)
     * @return true if movement started successfully
     */
    bool moveToAngles(const JointAngles& target, float feedrate = 0);
    
    /**
     * @brief Move to Cartesian position using IK
     * @param target Target Cartesian position
     * @param feedrate Movement speed (0 = use default maxSpeed)
     * @return true if movement successful
     */
    bool moveToPosition(const CartesianPos& target, float feedrate = 0);
    
    /**
     * @brief Get current joint angles (derived from actual motor step counts).
     *
     * Unlike the old implementation that cached the *target* angles,
     * this always reflects the true motor position — even mid-motion
     * or after an emergency stop.
     */
    JointAngles getCurrentAngles() const;
    
    /**
     * @brief Get current Cartesian position (FK from actual step counts)
     */
    CartesianPos getCurrentPosition() const;

    /**
     * @brief Get endstop state for axis index (0=T1, 1=T2, 2=T3)
     * @return true when endstop is triggered
     */
    bool getEndstopTriggered(uint8_t axisIndex) const;
    
    /**
     * @brief Check if motors are currently moving
     */
    bool isMoving() const { return moving; }
    
    /**
     * @brief Emergency stop - immediately halt all motion
     */
    void emergencyStop();
    
    /**
     * @brief Update motion state (call frequently in loop)
     */
    void update();

    /**
     * @brief Poll serial input for emergency stop triggers and stop immediately.
     * Triggers: '!' or Ctrl-X (0x18).
     * @return true if emergency stop was triggered by serial input.
     */
    bool serviceEmergencyStopInput();

    /**
     * @brief Consume emergency-stop latch raised by serial-triggered stop.
     * @return true if a serial emergency stop occurred since last consume.
     */
    bool consumeEmergencyStopLatch();

private:
    MotorConfig motors[3];
    MotionParams motion;
    Kinematics3D kinematics;
    uint8_t sharedEnablePin;
    
    // Current state (steps are the source of truth for position)
    long currentSteps[3];
    bool moving;
    bool enabled;
    
    // Target state for coordinated motion
    long targetSteps[3];
    float stepDelays[3];      // Cruise microseconds between steps for each motor
    unsigned long lastStepTime[3];
    long moveStartSteps[3];
    long leadAxisSteps;
    uint8_t leadAxisIndex;
    float moveRampPortion;
    float moveMinSpeedScale;

    // Endstop pins (theta1, theta2, theta3)
    uint8_t endstopPins[3];

    // Emergency input latch for blocking loops.
    bool emergencyStopLatched;
    
    // Helper functions
    long anglesToSteps(int motorIndex, float angleRad) const;
    float stepsToAngle(int motorIndex, long steps) const;
    void calculateCoordinatedMotion(const JointAngles& target, float feedrate);
    void stepMotor(int motorIndex);
    void stepMotorDirection(int motorIndex, int8_t direction);
    bool withinLimits(const JointAngles& angles) const;

    bool readEndstopRaw(int motorIndex) const;
    bool isEndstopTriggered(int motorIndex) const;
    int8_t thetaDirToStepDir(int motorIndex, int8_t thetaDirection) const;
    unsigned long homingStepDelayUs(int motorIndex, float feedrate) const;
    bool releasePressedEndstops(float feedrate);
    bool seekAllEndstops(float feedrate);
    bool moveHomeOffsetSteps(float feedrate);
    bool pollEmergencyStopInput();
};

#endif // NEMA17_H