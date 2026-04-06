#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>

// --- Gripper Configuration ---
struct GripperConfig {
    uint8_t in1Pin;
    uint8_t in2Pin;
    uint8_t in3Pin;
    uint8_t in4Pin;
    float stepsPerMM;          // Steps per millimeter of gripper movement
    float minPosition;         // Minimum position (mm) - fully open
    float maxPosition;         // Maximum position (mm) - fully closed
};

class BYJ48Gripper {
public:
    /**
     * @brief Initialize the gripper
     * @param config Gripper configuration
     */
    void begin(const GripperConfig& config);
    
    /**
     * @brief Move gripper to absolute position
     * @param position Target position in mm (0 = fully open, max = fully closed)
     * @param speed Speed in steps/second (1-500, default 300)
     * @return true if movement started successfully
     */
    bool moveToPosition(float position, float speed = 300.0);
    
    /**
     * @brief Move gripper relative to current position
     * @param distance Distance to move in mm (positive = close, negative = open)
     * @param speed Speed in steps/second (1-500, default 300)
     * @return true if movement started successfully
     */
    bool moveRelative(float distance, float speed = 300.0);
    
    /**
     * @brief Open gripper fully
     * @param speed Speed in steps/second (1-500, default 300)
     */
    void open(float speed = 300.0);
    
    /**
     * @brief Close gripper fully
     * @param speed Speed in steps/second (1-500, default 300)
     */
    void close(float speed = 300.0);
    
    /**
     * @brief Stop gripper immediately
     */
    void stop();
    
    /**
     * @brief Disable gripper motor (saves power, allows manual movement)
     */
    void disable();
    
    /**
     * @brief Enable gripper motor
     */
    void enable();
    
    /**
     * @brief Get current position in mm
     */
    float getCurrentPosition() const { return currentPosition; }
    
    /**
     * @brief Check if gripper is currently moving
     */
    bool isMoving() const { return moving; }
    
    /**
     * @brief Update gripper state (call frequently in loop)
     */
    void update();
    
    /**
     * @brief Set current position as zero reference
     */
    void setZero() { currentSteps = 0; currentPosition = 0; }
    
    /**
     * @brief Home the gripper (open fully then set as zero)
     * @param speed Homing speed in steps/second
     */
    void home(float speed = 200.0);

private:
    GripperConfig config;
    
    // Current state
    long currentSteps;
    float currentPosition;
    bool moving;
    bool enabled;
    
    // Target state
    long targetSteps;
    unsigned long stepDelay;
    unsigned long lastStepTime;
    int8_t direction;  // 1 = closing, -1 = opening
    
    // ULN2003 half-step sequence (8 steps per cycle)
    static const uint8_t STEP_SEQUENCE[8][4];
    uint8_t currentStep;
    
    // Helper functions
    void doStep();
    void setMotorPins(uint8_t step);
    long positionToSteps(float position);
    float stepsToPosition(long steps);
};

#endif // GRIPPER_H