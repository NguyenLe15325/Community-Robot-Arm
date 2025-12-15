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
    float acceleration;     // Acceleration in degrees/second²
    float jerk;            // Jerk limit (optional, for smoother motion)
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
     * @brief Disable all motors (reduces power consumption & heat)
     */
    void disable();
    
    /**
     * @brief Set motion parameters for coordinated movement
     */
    void setMotionParams(const MotionParams& params);
    
    /**
     * @brief Home the robot to initial position (0°, 90°, 0°)
     * @param feedrate Homing speed in degrees/second
     * @return true if homing successful
     */
    bool home(float feedrate = 30.0);
    
    /**
     * @brief Move to target joint angles with coordinated motion
     * @param target Target joint angles in radians
     * @param feedrate Movement speed (0 = use default maxSpeed)
     * @return true if movement successful
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
     * @brief Get current joint angles
     */
    JointAngles getCurrentAngles() const;
    
    /**
     * @brief Get current Cartesian position
     */
    CartesianPos getCurrentPosition() const;
    
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

private:
    MotorConfig motors[3];
    MotionParams motion;
    Kinematics3D kinematics;
    uint8_t sharedEnablePin;
    
    // Current state
    JointAngles currentAngles;
    long currentSteps[3];
    bool moving;
    bool enabled;
    
    // Target state for coordinated motion
    long targetSteps[3];
    float stepDelays[3];      // Microseconds between steps for each motor
    unsigned long lastStepTime[3];
    
    // Helper functions
    long anglesToSteps(int motorIndex, float angleRad);
    float stepsToAngle(int motorIndex, long steps);
    void calculateCoordinatedMotion(const JointAngles& target, float feedrate);
    void stepMotor(int motorIndex);
    bool withinLimits(const JointAngles& angles);
};

#endif // NEMA17_H