#ifndef GCODE_H
#define GCODE_H

#include <Arduino.h>
#include "NEMA17.h"
#include "Kinematics3D.h"
#include "Gripper.h"

// --- G-Code Command Structure ---
struct GCodeCommand {
    char command;           // G, M, etc.
    int number;            // Command number (e.g., 0 for G0)
    bool hasX, hasY, hasZ; // Cartesian coordinates present
    bool hasT1, hasT2, hasT3; // Joint angles present (custom)
    bool hasF;             // Feedrate present
    bool hasS;             // Spindle/Gripper parameter (reused for gripper position)
    float x, y, z;         // Cartesian values
    float t1, t2, t3;      // Joint angle values (degrees)
    float f;               // Feedrate
    float s;               // Gripper position (mm) or speed
};

class GCodeParser {
public:
    /**
     * @brief Initialize the G-Code parser
     * @param controller Reference to NEMA17 controller
     * @param gripperController Reference to gripper controller (optional, can be NULL)
     */
    void begin(NEMA17Controller* controller, BYJ48Gripper* gripperController = NULL);
    
    /**
     * @brief Process incoming serial data
     * Call this frequently in loop()
     */
    void update();
    
    /**
     * @brief Check if parser is busy (moving or delaying)
     */
    bool isBusy() const { return delaying; }
    
    /**
     * @brief Set verbose mode for debugging
     */
    void setVerbose(bool verbose) { verboseMode = verbose; }

private:
    NEMA17Controller* motor;
    BYJ48Gripper* gripper;
    bool verboseMode;
    String inputBuffer;
    
    // Current modal state
    bool absoluteMode;      // G90/G91
    float currentFeedrate;  // F value
    
    // Non-blocking delay state
    bool delaying;
    unsigned long delayStartTime;
    unsigned long delayDuration;
    
    // Helper functions
    bool parseLine(const String& line, GCodeCommand& cmd);
    bool executeCommand(const GCodeCommand& cmd);
    void sendResponse(const String& message);
    void sendError(const String& error);
    void sendOK();
    
    // Command handlers
    bool handleG0G1(const GCodeCommand& cmd);  // Linear move
    bool handleG4(const GCodeCommand& cmd);    // Dwell
    bool handleG28(const GCodeCommand& cmd);   // Home
    bool handleG90(const GCodeCommand& cmd);   // Absolute positioning
    bool handleG91(const GCodeCommand& cmd);   // Relative positioning
    bool handleM17(const GCodeCommand& cmd);   // Enable motors
    bool handleM18(const GCodeCommand& cmd);   // Disable motors
    bool handleM84(const GCodeCommand& cmd);   // Disable motors (alias)
    bool handleM3(const GCodeCommand& cmd);    // Gripper close (with S parameter)
    bool handleM5(const GCodeCommand& cmd);    // Gripper open
    bool handleM6(const GCodeCommand& cmd);    // Gripper home
    bool handleM114(const GCodeCommand& cmd);  // Get current position
    bool handleM400(const GCodeCommand& cmd);  // Wait for moves to finish
    
    void printHelp();  // Print comprehensive command guide
};

#endif // GCODE_H