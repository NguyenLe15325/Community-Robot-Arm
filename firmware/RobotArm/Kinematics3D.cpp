#include "Kinematics3D.h"
#include <math.h>

// --- Static Helpers ---

bool Kinematics3D::checkLimits(const JointAngles& angles) {
    if (angles.theta1 < THETA1_MIN_RAD || angles.theta1 > THETA1_MAX_RAD) return false;
    if (angles.theta2 < THETA2_MIN_RAD || angles.theta2 > THETA2_MAX_RAD) return false;
    if (angles.theta3 < THETA3_MIN_RAD || angles.theta3 > THETA3_MAX_RAD) return false;
    return true;
}

float Kinematics3D::normalizeAngle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float Kinematics3D::calculateTheta2(float t3, float k1, float k2) {
    float c2 = k1 - cos(t3);
    float s2 = k2 + sin(t3);
    if (fabsf(c2 * c2 + s2 * s2 - 1.0f) > 1e-4f) {
        return NAN;
    }
    return atan2(s2, c2);
}

// --- Forward Kinematics Implementation ---
CartesianPos Kinematics3D::forwardKinematics(const JointAngles& angles) const {
    CartesianPos pos;
    float c1 = cos(angles.theta1);
    float s1 = sin(angles.theta1);
    float c2 = cos(angles.theta2);
    float s2 = sin(angles.theta2);
    float c3 = cos(angles.theta3);
    float s3 = sin(angles.theta3);

    // Common term R = L*cos(theta2) + L*cos(theta3) + a
    float R = (L * c2) + (L * c3) + a;

    pos.x = R * c1;
    pos.y = (L * s2) - (L * s3);
    pos.z = R * s1;

    return pos;
}

// --- Inverse Kinematics Implementation ---
bool Kinematics3D::inverseKinematics(const CartesianPos& pos, JointAngles& result) const {
    
    // --- 0. Initial Cartesian Limit Check ---
    if (pos.x > X_MAX || pos.x < X_MIN || 
        pos.z > Z_MAX || pos.z < Z_MIN || 
        pos.y > Y_MAX || pos.y < Y_MIN) {
        return false; 
    }

    // 1. Solve for theta1
    float R_sq = pos.x * pos.x + pos.z * pos.z;
    float R = sqrt(R_sq);

    if (R > R_MAX_REACH || R < R_MIN_REACH) {
        return false; 
    }
    
    if (R < 1e-4f) { 
        result.theta1 = 0.0f;
    } else {
        result.theta1 = atan2(pos.z, pos.x);
    }
    
    if (result.theta1 < THETA1_MIN_RAD || result.theta1 > THETA1_MAX_RAD) {
        return false;
    }

    // 2. Solve for theta2 and theta3
    float K1 = (R - a) / L;
    float K2 = pos.y / L;
    
    float A = -2.0f * K1;
    float B = 2.0f * K2;
    float C = -(K1 * K1 + K2 * K2);

    float R_aux_sq = A * A + B * B;
    if (R_aux_sq < 1e-6f) {
        if (fabsf(R - a) < 1e-4f && fabsf(pos.y) < 1e-4f) {
            result.theta2 = THETA2_MIN_RAD; 
            result.theta3 = THETA3_MIN_RAD; 
            if (checkLimits(result)) return true;
        }
        return false; 
    }
    float R_aux = sqrt(R_aux_sq);

    if (fabsf(C) > R_aux + 1e-4f) { 
        return false; 
    }

    float alpha = atan2(B, A);
    float phi = acos(C / R_aux);

    float theta3_sol1 = alpha + phi;
    float theta3_sol2 = alpha - phi;

    // Try Solution 1 (Normalized)
    result.theta3 = normalizeAngle(theta3_sol1);
    result.theta2 = calculateTheta2(result.theta3, K1, K2);
    
    if (!isnan(result.theta2) && checkLimits(result)) {
        return true; 
    }

    // Try Solution 2 (Normalized)
    result.theta3 = normalizeAngle(theta3_sol2);
    result.theta2 = calculateTheta2(result.theta3, K1, K2);

    if (!isnan(result.theta2) && checkLimits(result)) {
        return true; 
    }

    return false;
}