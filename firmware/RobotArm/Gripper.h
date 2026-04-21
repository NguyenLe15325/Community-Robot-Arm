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
     * @brief Homing phase state machine.
     */
    enum HomingPhase : uint8_t {
        HOMING_NONE = 0,
        HOMING_CLOSE,   // Phase 1: close to find mechanical stop
        HOMING_OPEN     // Phase 2: open to known position
    };

    /**
     * @brief Initialize the gripper
     * @param config Gripper configuration
     */
    void begin(const GripperConfig& config);
    
    /**
     * @brief Move gripper to absolute position
     * @param position Target position in mm (0 = fully open, max = fully closed)
     * @param speed Speed in steps/second (1-1700, default 300)
     * @return true if movement started successfully
     */
    bool moveToPosition(float position, float speed = 300.0f);
    
    /**
     * @brief Move gripper relative to current position
     * @param distance Distance to move in mm (positive = close, negative = open)
     * @param speed Speed in steps/second (1-1700, default 300)
     * @return true if movement started successfully
     */
    bool moveRelative(float distance, float speed = 300.0f);
    
    /**
     * @brief Open gripper by a relative amount (firmware default)
     * @param speed Speed in steps/second (1-1700, default 300)
     * @note Default distance moved is GRIPPER_RELATIVE_MOVE_MM (mm)
     */
    void open(float speed = 300.0f);
    
    /**
     * @brief Close gripper by a relative amount (firmware default)
     * @param speed Speed in steps/second (1-1700, default 300)
     * @note Default distance moved is GRIPPER_RELATIVE_MOVE_MM (mm)
     */
    void close(float speed = 300.0f);

    /**
     * @brief Start the 2-phase homing sequence (close then open).
     *
     * The homing state machine runs inside update(). isMoving() returns
     * true for the entire duration.  The caller should treat the command
     * as asynchronous and wait for isMoving() == false.
     *
     * @param speedStepsPerSec Homing speed in motor steps/second.
     */
    void startHome(float speedStepsPerSec);
    
    /**
     * @brief Stop gripper immediately and cancel any homing in progress
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
     * @brief Get approximate current position in mm.
     * @note Position is derived from step count; not guaranteed accurate
     *       in relative-only mode.
     */
    float getCurrentPosition() const;
    
    /**
     * @brief Check if gripper is currently moving (includes homing phases)
     */
    bool isMoving() const { return moving || (homingPhase != HOMING_NONE); }

    /**
     * @brief Check if gripper is in homing sequence
     */
    bool isHoming() const { return homingPhase != HOMING_NONE; }
    
    /**
     * @brief Update gripper state (call frequently in loop)
     */
    void update();
    
    /**
     * @brief Set current position as zero reference
     * @note Deprecated for relative-only firmware.
     */
    void setZero() { currentSteps = 0; }

    /**
     * @brief Convert gripper linear speed from mm/s to motor steps/s.
     * @param speedMmPerSec Requested linear speed in mm/s.
     * @return Equivalent speed in steps/second (non-negative).
     */
    float mmPerSecToStepsPerSec(float speedMmPerSec) const;

private:
    GripperConfig config;
    
    // Current state
    long currentSteps;
    bool moving;
    bool enabled;
    
    // Target state
    long targetSteps;
    unsigned long stepDelay;
    unsigned long lastStepTime;
    int8_t direction;  // 1 = closing, -1 = opening

    // Homing state machine
    HomingPhase homingPhase;
    float homingSpeed;
    
    // ULN2003 half-step sequence (8 steps per cycle)
    static const uint8_t STEP_SEQUENCE[8][4];
    uint8_t currentStep;
    
    // Helper functions
    void doStep();
    void setMotorPins(uint8_t step);
    long positionToSteps(float position);

    /**
     * @brief Clamp speed to safe BYJ-48 stepper range.
     */
    static float clampSpeed(float speed);
};

#endif // GRIPPER_H