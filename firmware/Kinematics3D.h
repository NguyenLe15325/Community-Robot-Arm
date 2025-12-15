#ifndef KINEMATICS3D_H
#define KINEMATICS3D_H

#include <Arduino.h>

// --- Configurable Dimensions (in mm) ---
const float L = 140.0; // Link length
const float a = 54.0;  // Offset/Base length

// --- Joint Limits (converted to radians) ---
const float THETA1_MAX_RAD = 90.0 * (M_PI / 180.0);
const float THETA1_MIN_RAD = -90.0 * (M_PI / 180.0);
const float THETA2_MAX_RAD = 130.0 * (M_PI / 180.0);
const float THETA2_MIN_RAD = 0.0 * (M_PI / 180.0);
const float THETA3_MAX_RAD = 90.0 * (M_PI / 180.0);
const float THETA3_MIN_RAD = -17.0 * (M_PI / 180.0);

// --- Position Limits (in mm) ---

// User-specified limits
const float X_MAX = 320.0;
const float X_MIN = 0;
const float Z_MAX = 320.0;
const float Z_MIN = -320.0;

// Derived limits from user specifications:
// Y_MAX = L - L*sin(theta3min)
const float Y_MAX = L - L * sin(THETA3_MIN_RAD);

// Y_MIN = L*sin(theta2min) - L*sin(theta3max)
const float Y_MIN = L * sin(THETA2_MIN_RAD) - L * sin(THETA3_MAX_RAD);

// Calculated Radial Reach Limits (Used internally for robust IK)
// R_MIN_REACH = L*cos(theta2max) + L*cos(theta3min) + a
const float R_MIN_REACH = L * cos(THETA2_MAX_RAD) + L * cos(THETA3_MIN_RAD) + a;
const float R_MAX_REACH = 320;

/**
 * @brief Structure to hold the joint angles (in radians).
 */
typedef struct {
    float theta1;
    float theta2;
    float theta3;
} JointAngles;

/**
 * @brief Structure to hold the Cartesian coordinates (in mm).
 */
typedef struct {
    float x;
    float y;
    float z;
} CartesianPos;

class Kinematics3D {
public:
    /**
     * @brief Computes the Cartesian position from joint angles (Forward Kinematics).
     */
    CartesianPos forwardKinematics(const JointAngles& angles);

    /**
     * @brief Computes the joint angles from a Cartesian position (Inverse Kinematics).
     * @return true if a valid solution was found, false otherwise.
     */
    bool inverseKinematics(const CartesianPos& pos, JointAngles& result);

private:
    /**
     * @brief Checks if the calculated joint angles are within the defined limits.
     */
    bool checkLimits(const JointAngles& angles);
};

#endif // KINEMATIC3D_H