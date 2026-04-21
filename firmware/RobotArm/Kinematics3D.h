#ifndef KINEMATICS3D_H
#define KINEMATICS3D_H

#include <Arduino.h>

// --- Configurable Dimensions (in mm) ---
constexpr float L = 140.0f; // Link length
constexpr float a = 54.0f;  // Offset/Base length

// --- Joint Limits (converted to radians) ---
constexpr float THETA1_MAX_RAD = 90.0f * (M_PI / 180.0f);
constexpr float THETA1_MIN_RAD = -90.0f * (M_PI / 180.0f);
constexpr float THETA2_MAX_RAD = 130.0f * (M_PI / 180.0f);
constexpr float THETA2_MIN_RAD = 0.0f * (M_PI / 180.0f);
constexpr float THETA3_MAX_RAD = 120.0f * (M_PI / 180.0f);
constexpr float THETA3_MIN_RAD = -17.0f * (M_PI / 180.0f);

// --- Position Limits (in mm) ---

// User-specified limits
constexpr float X_MAX = 320.0f;
constexpr float X_MIN = 0.0f;
constexpr float Z_MAX = 320.0f;
constexpr float Z_MIN = -320.0f;

// Derived limits from user specifications:
// Y_MAX = L - L*sin(theta3min)
const float Y_MAX = L - L * sin(THETA3_MIN_RAD);

// Y_MIN = L*sin(theta2min) - L*sin(theta3max)
const float Y_MIN = L * sin(THETA2_MIN_RAD) - L * sin(THETA3_MAX_RAD);

// Calculated Radial Reach Limits (Used internally for robust IK)
// R_MIN_REACH = L*cos(theta2max) + L*cos(theta3min) + a
const float R_MIN_REACH = L * cos(THETA2_MAX_RAD) + L * cos(THETA3_MIN_RAD) + a;
constexpr float R_MAX_REACH = 320.0f;

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
    CartesianPos forwardKinematics(const JointAngles& angles) const;

    /**
     * @brief Computes the joint angles from a Cartesian position (Inverse Kinematics).
     * @return true if a valid solution was found, false otherwise.
     */
    bool inverseKinematics(const CartesianPos& pos, JointAngles& result) const;

private:
    /**
     * @brief Checks if the calculated joint angles are within the defined limits.
     */
    static bool checkLimits(const JointAngles& angles);

    /**
     * @brief Normalize angle to [-PI, PI] range.
     */
    static float normalizeAngle(float angle);

    /**
     * @brief Calculate theta2 from theta3 and intermediate values.
     * @return theta2 in radians, or NAN if inconsistent.
     */
    static float calculateTheta2(float t3, float k1, float k2);
};

#endif // KINEMATICS3D_H