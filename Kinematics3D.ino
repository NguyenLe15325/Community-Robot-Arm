#include "Kinematics3D.h"
// This assumes Kinematics3D.h and Kinematics3D.cpp are in the same folder as this sketch.

Kinematics3D arm;
JointAngles resultAngles;
CartesianPos resultPos;

// --- IMPORTANT: Update these constants based on your measurements ---
// These values override the theoretical R limits used in Kinematics3D.h for this demo.
// Ensure your Kinematics3D.h is set up to accept your measured R limits (as provided previously).
const float DEMO_R_MIN_REACH = 150.0;
const float DEMO_R_MAX_REACH = 320.0;
// Note: We use the header's constant names here for clarity, though they are technically defined there.
// For this single .ino file demo, we assume the Kinematics3D.h already has these values defined.

// Helper function to print a JointAngles struct
void printAngles(const JointAngles& angles, const char* label) {
    Serial.print(label);
    Serial.print(" (rad): T1="); Serial.print(angles.theta1, 4);
    Serial.print(", T2="); Serial.print(angles.theta2, 4);
    Serial.print(", T3="); Serial.print(angles.theta3, 4);
    
    float deg1 = angles.theta1 * (180.0 / M_PI);
    float deg2 = angles.theta2 * (180.0 / M_PI);
    float deg3 = angles.theta3 * (180.0 / M_PI);
    Serial.print(" | (deg): T1="); Serial.print(deg1, 2);
    Serial.print(", T2="); Serial.print(deg2, 2);
    Serial.print(", T3="); Serial.println(deg3, 2);
}

// Helper function to print a CartesianPos struct
void printPosition(const CartesianPos& pos, const char* label) {
    Serial.print(label);
    Serial.print(" (mm): X="); Serial.print(pos.x, 2);
    Serial.print(", Y="); Serial.print(pos.y, 2);
    Serial.print(", Z="); Serial.println(pos.z, 2);
}

// --- TEST FUNCTION ---
void runIKFKTest(const char* name, const CartesianPos& target) {
    Serial.println();
    Serial.print("--- Running Test: ");
    Serial.print(name);
    Serial.println(" ---");
    printPosition(target, "Target Pos");

    // Attempt Inverse Kinematics
    if (arm.inverseKinematics(target, resultAngles)) {
        Serial.println("✅ IK Succeeded. Solution Found:");
        printAngles(resultAngles, "Calculated Angles");

        // Validate solution using Forward Kinematics
        resultPos = arm.forwardKinematics(resultAngles);
        Serial.println("🚀 FK Validation:");
        printPosition(resultPos, "Calculated Pos");
        
        // Simple validation check
        float error_x = abs(resultPos.x - target.x);
        float error_y = abs(resultPos.y - target.y);
        float error_z = abs(resultPos.z - target.z);
        
        if (error_x < 0.1 && error_y < 0.1 && error_z < 0.1) {
            Serial.println("✨ Validation successful (Error < 0.1mm).");
        } else {
            Serial.println("⚠️ Validation FAILED (Large error detected).");
        }

    } else {
        Serial.println("❌ IK Failed. Target is unreachable or outside limits.");
    }
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    while (!Serial) {;} // Wait for serial port to connect (Leonardo/Micro)
    Serial.println("\n==============================================");
    Serial.println("     Kinematics 3D Test Demo (R_min=150, R_max=320)    ");
    Serial.println("==============================================");
    
    // 1. STANDARD REACH TEST 🎯
    CartesianPos standardTarget = {0.0, 140.0, 194.0};
    runIKFKTest("1. Standard Reach Test (R ~ 224mm)", standardTarget);

    // 2. MAXIMUM REACH TEST (R_MAX = 320mm) 💪
    CartesianPos maxReachTarget = {DEMO_R_MAX_REACH * cos(10.0 * (M_PI / 180.0)), 0.0, DEMO_R_MAX_REACH * sin(10.0 * (M_PI / 180.0))};
    runIKFKTest("2. Max Radial Reach (R=320mm, Should Succeed)", maxReachTarget);

    // 3. OVER-EXTENSION FAILURE TEST (R > 320mm) 🚫
    CartesianPos overExtendTarget = {DEMO_R_MAX_REACH + 5.0, 0.0, 0.0};
    runIKFKTest("3. Over-Extension Failure (R=325mm, Should Fail)", overExtendTarget);
    
    // 4. MINIMUM REACH TEST (R_MIN = 150mm) 🤏
    CartesianPos minReachTarget = {DEMO_R_MIN_REACH, 0.0, 0.0}; 
    runIKFKTest("4. Min Radial Reach (R=150mm, Should Succeed)", minReachTarget);

    // 5. UNDER-REACH FAILURE TEST (R < 150mm) 🛑
    CartesianPos underReachTarget = {DEMO_R_MIN_REACH - 50.0, 0.0, 0.0}; // R=100mm
    runIKFKTest("5. Under-Reach Failure (R=100mm, Should Fail)", underReachTarget);

    // 6. JOINT LIMIT FAILURE TEST (Requesting a point requiring T1 > 90 deg) 📐
    CartesianPos angleLimitFailTarget = {10.0 * cos(100.0 * (M_PI / 180.0)), 0.0, 10.0 * sin(100.0 * (M_PI / 180.0))};
    runIKFKTest("6. Joint Limit Failure (Requires T1=100 deg)", angleLimitFailTarget);
    
    Serial.println("\n==============================================");
    Serial.println("          Demo Complete          ");
    Serial.println("==============================================");
}

void loop() {
    // Demo only runs once in setup.
}