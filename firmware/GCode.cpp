#include "GCode.h"

// --- Initialization ---
void GCodeParser::begin(NEMA17Controller* controller, BYJ48Gripper* gripperController) {
    motor = controller;
    gripper = gripperController;
    verboseMode = false;
    inputBuffer = "";
    absoluteMode = true;
    currentFeedrate = 60.0; // Default 60 deg/s or mm/s depending on mode
    
    // Initialize delay state
    delaying = false;
    delayStartTime = 0;
    delayDuration = 0;
    
    Serial.println(F("G-Code Parser Ready"));
    Serial.println(F("Supported: G0/G1, G4, G28, G90/G91, M17/M18/M84, M114, M400"));
    Serial.println(F("Custom: T1/T2/T3 for joint angles (degrees)"));
    if (gripper) {
        Serial.println(F("Gripper: M3 (close), M5 (open), M3 S<pos> (position)"));
    }
    sendOK();
}

// --- Update Loop ---
void GCodeParser::update() {
    // Check if we're in a non-blocking delay
    if (delaying) {
        if (millis() - delayStartTime >= delayDuration) {
            delaying = false;
            if (verboseMode) {
                Serial.println(F("Delay complete"));
            }
            sendOK();
        }
        return; // Don't process new commands while delaying
    }
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                GCodeCommand cmd;
                if (parseLine(inputBuffer, cmd)) {
                    if (executeCommand(cmd)) {
                        // Only send OK immediately if not delaying
                        if (!delaying) {
                            sendOK();
                        }
                    }
                } else {
                    sendError("Parse error");
                }
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
            if (inputBuffer.length() > 128) { // Prevent buffer overflow
                sendError("Line too long");
                inputBuffer = "";
            }
        }
    }
}

// --- Response Functions ---
void GCodeParser::sendResponse(const String& message) {
    Serial.println(message);
}

void GCodeParser::sendError(const String& error) {
    Serial.print(F("Error: "));
    Serial.println(error);
}

void GCodeParser::sendOK() {
    Serial.println(F("ok"));
}

// --- Parser ---
bool GCodeParser::parseLine(const String& line, GCodeCommand& cmd) {
    // Initialize command
    cmd.command = '\0';
    cmd.number = -1;
    cmd.hasX = cmd.hasY = cmd.hasZ = false;
    cmd.hasT1 = cmd.hasT2 = cmd.hasT3 = false;
    cmd.hasF = false;
    cmd.hasS = false;
    cmd.x = cmd.y = cmd.z = 0;
    cmd.t1 = cmd.t2 = cmd.t3 = 0;
    cmd.f = 0;
    cmd.s = 0;
    
    String processedLine = line;
    processedLine.trim();
    processedLine.toUpperCase();
    
    // Remove comments
    int commentPos = processedLine.indexOf(';');
    if (commentPos >= 0) {
        processedLine = processedLine.substring(0, commentPos);
        processedLine.trim();
    }
    
    if (processedLine.length() == 0) return false;
    
    // Parse command letter and number
    if (processedLine[0] == 'G' || processedLine[0] == 'M') {
        cmd.command = processedLine[0];
        int i = 1;
        String numStr = "";
        while (i < processedLine.length() && isDigit(processedLine[i])) {
            numStr += processedLine[i];
            i++;
        }
        if (numStr.length() > 0) {
            cmd.number = numStr.toInt();
        }
    } else if (processedLine.startsWith("HELP")) {
        // Special help command
        cmd.command = 'H';
        cmd.number = 0;
        return true;
    }
    
    // Parse parameters
    int pos = 0;
    while (pos < processedLine.length()) {
        char param = processedLine[pos];
        
        if (param == 'X' || param == 'Y' || param == 'Z' || 
            param == 'F' || param == 'T' || param == 'P' || param == 'S') {
            pos++;
            String valueStr = "";
            
            // Handle T1, T2, T3
            if (param == 'T' && pos < processedLine.length() && isDigit(processedLine[pos])) {
                char jointNum = processedLine[pos];
                pos++;
                while (pos < processedLine.length() && 
                       (isDigit(processedLine[pos]) || processedLine[pos] == '.' || 
                        processedLine[pos] == '-')) {
                    valueStr += processedLine[pos];
                    pos++;
                }
                float value = valueStr.toFloat();
                if (jointNum == '1') { cmd.hasT1 = true; cmd.t1 = value; }
                else if (jointNum == '2') { cmd.hasT2 = true; cmd.t2 = value; }
                else if (jointNum == '3') { cmd.hasT3 = true; cmd.t3 = value; }
                continue;
            }
            
            // Parse numeric value
            while (pos < processedLine.length() && 
                   (isDigit(processedLine[pos]) || processedLine[pos] == '.' || 
                    processedLine[pos] == '-')) {
                valueStr += processedLine[pos];
                pos++;
            }
            
            if (valueStr.length() > 0) {
                float value = valueStr.toFloat();
                switch (param) {
                    case 'X': cmd.hasX = true; cmd.x = value; break;
                    case 'Y': cmd.hasY = true; cmd.y = value; break;
                    case 'Z': cmd.hasZ = true; cmd.z = value; break;
                    case 'F': cmd.hasF = true; cmd.f = value; break;
                    case 'S': cmd.hasS = true; cmd.s = value; break;
                    case 'P': cmd.hasF = true; cmd.f = value; break; // P for dwell time
                }
            }
        } else {
            pos++;
        }
    }
    
    return cmd.command != '\0';
}

// --- Command Executor ---
bool GCodeParser::executeCommand(const GCodeCommand& cmd) {
    // Handle help command
    if (cmd.command == 'H') {
        printHelp();
        return true;
    }
    
    if (cmd.command == 'G') {
        switch (cmd.number) {
            case 0:
            case 1: return handleG0G1(cmd);
            case 4: return handleG4(cmd);
            case 28: return handleG28(cmd);
            case 90: return handleG90(cmd);
            case 91: return handleG91(cmd);
            default:
                sendError("Unsupported G command: G" + String(cmd.number));
                return false;
        }
    } else if (cmd.command == 'M') {
        switch (cmd.number) {
            case 3: return handleM3(cmd);
            case 5: return handleM5(cmd);
            case 6: return handleM6(cmd);
            case 17: return handleM17(cmd);
            case 18: return handleM18(cmd);
            case 84: return handleM84(cmd);
            case 114: return handleM114(cmd);
            case 400: return handleM400(cmd);
            case 3001: return handleM3001(cmd);
            default:
                sendError("Unsupported M command: M" + String(cmd.number));
                return false;
        }
    }
    
    sendError("Unknown command");
    return false;
}

// --- Command Handlers ---
bool GCodeParser::handleG0G1(const GCodeCommand& cmd) {
    // Update feedrate if provided
    if (cmd.hasF) {
        currentFeedrate = cmd.f;
    }
    
    // Check if we have joint angles (T1/T2/T3) or Cartesian (X/Y/Z)
    if (cmd.hasT1 || cmd.hasT2 || cmd.hasT3) {
        // Joint angle mode
        JointAngles target = motor->getCurrentAngles();
        
        if (cmd.hasT1) target.theta1 = cmd.t1 * (M_PI / 180.0);
        if (cmd.hasT2) target.theta2 = cmd.t2 * (M_PI / 180.0);
        if (cmd.hasT3) target.theta3 = cmd.t3 * (M_PI / 180.0);
        
        if (verboseMode) {
            Serial.print(F("Moving to joints: T1="));
            Serial.print(target.theta1 * 180.0 / M_PI);
            Serial.print(F(" T2="));
            Serial.print(target.theta2 * 180.0 / M_PI);
            Serial.print(F(" T3="));
            Serial.println(target.theta3 * 180.0 / M_PI);
        }
        
        if (!motor->moveToAngles(target, currentFeedrate)) {
            sendError("Joint movement failed (limits exceeded?)");
            return false;
        }
        
    } else if (cmd.hasX || cmd.hasY || cmd.hasZ) {
        // Cartesian mode
        CartesianPos target = motor->getCurrentPosition();
        
        if (absoluteMode) {
            if (cmd.hasX) target.x = cmd.x;
            if (cmd.hasY) target.y = cmd.y;
            if (cmd.hasZ) target.z = cmd.z;
        } else {
            // Relative mode
            if (cmd.hasX) target.x += cmd.x;
            if (cmd.hasY) target.y += cmd.y;
            if (cmd.hasZ) target.z += cmd.z;
        }
        
        if (verboseMode) {
            Serial.print(F("Moving to: X="));
            Serial.print(target.x);
            Serial.print(F(" Y="));
            Serial.print(target.y);
            Serial.print(F(" Z="));
            Serial.println(target.z);
        }
        
        if (!motor->moveToPosition(target, currentFeedrate)) {
            sendError("Cartesian movement failed (unreachable or IK failed)");
            return false;
        }
        
    } else {
        sendError("No coordinates provided");
        return false;
    }
    
    return true;
}

bool GCodeParser::handleG4(const GCodeCommand& cmd) {
    // Dwell - non-blocking wait for specified time
    if (!cmd.hasF) {
        sendError("G4 requires P parameter (milliseconds)");
        return false;
    }
    
    delayDuration = (unsigned long)cmd.f;
    
    if (delayDuration == 0) {
        return true; // No delay needed
    }
    
    if (verboseMode) {
        Serial.print(F("Delaying for "));
        Serial.print(delayDuration);
        Serial.println(F("ms (non-blocking)"));
    }
    
    delaying = true;
    delayStartTime = millis();
    
    // Don't send OK yet - will send when delay completes
    return true;
}

bool GCodeParser::handleG28(const GCodeCommand& cmd) {
    // Home the robot
    if (verboseMode) {
        Serial.println(F("Homing..."));
    }
    
    if (!motor->home(currentFeedrate)) {
        sendError("Homing failed");
        return false;
    }
    
    // Wait for homing to complete
    while (motor->isMoving()) {
        motor->update();
        delay(1);
    }
    
    if (gripper) {
        while (gripper->isMoving()) {
            gripper->update();
            delay(1);
        }
    }
    
    if (verboseMode) {
        Serial.println(F("Homing complete"));
    }
    
    return true;
}

bool GCodeParser::handleG90(const GCodeCommand& cmd) {
    absoluteMode = true;
    if (verboseMode) {
        Serial.println(F("Absolute positioning mode"));
    }
    return true;
}

bool GCodeParser::handleG91(const GCodeCommand& cmd) {
    absoluteMode = false;
    if (verboseMode) {
        Serial.println(F("Relative positioning mode"));
    }
    return true;
}

bool GCodeParser::handleM17(const GCodeCommand& cmd) {
    motor->enable();
    if (verboseMode) {
        Serial.println(F("Motors enabled"));
    }
    return true;
}

bool GCodeParser::handleM18(const GCodeCommand& cmd) {
    motor->disable();
    if (gripper) gripper->disable();
    if (verboseMode) {
        Serial.println(F("Motors disabled"));
    }
    return true;
}

bool GCodeParser::handleM84(const GCodeCommand& cmd) {
    return handleM18(cmd); // Alias for M18
}

bool GCodeParser::handleM3(const GCodeCommand& cmd) {
    // M3: Close gripper or move to position
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    float speed = 300.0; // Default speed
    if (cmd.hasF) speed = cmd.f;
    
    if (cmd.hasS) {
        // Move to specific position
        if (verboseMode) {
            Serial.print(F("Gripper to position: "));
            Serial.print(cmd.s);
            Serial.println(F("mm"));
        }
        gripper->moveToPosition(cmd.s, speed);
    } else {
        // Close gripper fully
        if (verboseMode) {
            Serial.println(F("Closing gripper"));
        }
        gripper->close(speed);
    }
    
    return true;
}

bool GCodeParser::handleM5(const GCodeCommand& cmd) {
    // M5: Open gripper
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    float speed = 300.0; // Default speed
    if (cmd.hasF) speed = cmd.f;
    
    if (verboseMode) {
        Serial.println(F("Opening gripper"));
    }
    gripper->open(speed);
    
    return true;
}

bool GCodeParser::handleM6(const GCodeCommand& cmd) {
    // M6: Home gripper
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    float speed = 200.0; // Default homing speed
    if (cmd.hasF) speed = cmd.f;
    
    if (verboseMode) {
        Serial.println(F("Homing gripper..."));
    }
    
    gripper->home(speed);
    
    // Wait for homing to complete
    while (gripper->isMoving()) {
        gripper->update();
        delay(1);
    }
    
    // Set current position as zero
    gripper->setZero();
    
    if (verboseMode) {
        Serial.println(F("Gripper homed (position set to 0)"));
    }
    
    return true;
}

bool GCodeParser::handleM114(const GCodeCommand& cmd) {
    // Report current position
    CartesianPos pos = motor->getCurrentPosition();
    JointAngles angles = motor->getCurrentAngles();
    
    Serial.print(F("X:"));
    Serial.print(pos.x, 2);
    Serial.print(F(" Y:"));
    Serial.print(pos.y, 2);
    Serial.print(F(" Z:"));
    Serial.print(pos.z, 2);
    Serial.print(F(" T1:"));
    Serial.print(angles.theta1 * 180.0 / M_PI, 2);
    Serial.print(F(" T2:"));
    Serial.print(angles.theta2 * 180.0 / M_PI, 2);
    Serial.print(F(" T3:"));
    Serial.println(angles.theta3 * 180.0 / M_PI, 2);
    
    if (gripper) {
        Serial.print(F("Gripper: "));
        Serial.print(gripper->getCurrentPosition(), 2);
        Serial.println(F("mm"));
    }
    
    return true;
}

bool GCodeParser::handleM400(const GCodeCommand& cmd) {
    // Wait for all moves to finish
    if (verboseMode) {
        Serial.println(F("Waiting for moves to complete..."));
    }
    
    while (motor->isMoving()) {
        motor->update();
        delay(1);
    }
    
    if (verboseMode) {
        Serial.println(F("All moves complete"));
    }
    
    return true;
}

bool GCodeParser::handleM3001(const GCodeCommand& cmd) {
    // Report gripper position only
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    Serial.print(F("Gripper: "));
    Serial.print(gripper->getCurrentPosition(), 2);
    Serial.print(F("mm"));
    
    if (gripper->isMoving()) {
        Serial.print(F(" (moving)"));
    }
    
    Serial.println();
    
    return true;
}

// --- Help Function ---
void GCodeParser::printHelp() {
    Serial.println(F("\n========================================"));
    Serial.println(F("         COMMAND REFERENCE"));
    Serial.println(F("========================================"));
    
    Serial.println(F("\n--- MOTION COMMANDS ---"));
    Serial.println(F("G0/G1 X<pos> Y<pos> Z<pos> F<speed>"));
    Serial.println(F("  Move to Cartesian position (mm)"));
    Serial.println(F("  Example: G0 X200 Y50 Z100 F60"));
    Serial.println();
    Serial.println(F("G0/G1 T1<deg> T2<deg> T3<deg> F<speed>"));
    Serial.println(F("  Move to joint angles (degrees)"));
    Serial.println(F("  Example: G0 T10 T245 T30 F90"));
    Serial.println();
    Serial.println(F("G4 P<ms>"));
    Serial.println(F("  Non-blocking delay in milliseconds"));
    Serial.println(F("  Example: G4 P500 (wait 0.5 seconds)"));
    Serial.println();
    Serial.println(F("G28"));
    Serial.println(F("  Home robot to initial position"));
    Serial.println(F("  (Theta1=0, Theta2=90, Theta3=0)"));
    Serial.println();
    Serial.println(F("G90"));
    Serial.println(F("  Absolute positioning mode (default)"));
    Serial.println();
    Serial.println(F("G91"));
    Serial.println(F("  Relative positioning mode"));
    
    if (gripper) {
        Serial.println(F("\n--- GRIPPER COMMANDS ---"));
        Serial.println(F("M3"));
        Serial.println(F("  Close gripper fully"));
        Serial.println(F("  Example: M3 F400 (with custom speed)"));
        Serial.println();
        Serial.println(F("M3 S<pos>"));
        Serial.println(F("  Move gripper to position (0-30mm)"));
        Serial.println(F("  Example: M3 S15 (half closed)"));
        Serial.println();
        Serial.println(F("M5"));
        Serial.println(F("  Open gripper fully"));
        Serial.println();
        Serial.println(F("M6"));
        Serial.println(F("  Home gripper (open & set zero)"));
    }
    
    Serial.println(F("\n--- SYSTEM COMMANDS ---"));
    Serial.println(F("M17"));
    Serial.println(F("  Enable all motors"));
    Serial.println();
    Serial.println(F("M18 / M84"));
    Serial.println(F("  Disable all motors"));
    Serial.println();
    Serial.println(F("M114"));
    Serial.println(F("  Report current position"));
    Serial.println(F("  (Cartesian + joint angles + gripper)"));
    Serial.println();
    Serial.println(F("M400"));
    Serial.println(F("  Wait for all moves to finish"));
    Serial.println();
    if (gripper) {
        Serial.println(F("M3001"));
        Serial.println(F("  Report gripper position only"));
        Serial.println();
    }
    Serial.println(F("HELP"));
    Serial.println(F("  Display this command reference"));
    
    Serial.println(F("\n--- JOINT LIMITS ---"));
    Serial.println(F("  Theta1 (Base):     -90° to +90°"));
    Serial.println(F("  Theta2 (Shoulder):   0° to 130°"));
    Serial.println(F("  Theta3 (Elbow):    -17° to +90°"));
    
    Serial.println(F("\n--- EXAMPLE SEQUENCES ---"));
    Serial.println(F("\n1. Basic Startup:"));
    Serial.println(F("   M17              ; Enable motors"));
    Serial.println(F("   G28              ; Home robot"));
    if (gripper) {
        Serial.println(F("   M6               ; Home gripper"));
    }
    
    Serial.println(F("\n2. Simple Movement:"));
    Serial.println(F("   G0 X200 Y50 Z100 F60"));
    Serial.println(F("   M400             ; Wait"));
    Serial.println(F("   G0 T10 T245 T30  ; Joint mode"));
    
    if (gripper) {
        Serial.println(F("\n3. Pick and Place:"));
        Serial.println(F("   G0 X200 Y100 Z50 ; Move to pick"));
        Serial.println(F("   M400             ; Wait for motion"));
        Serial.println(F("   G4 P200          ; Stabilize"));
        Serial.println(F("   M5               ; Open gripper"));
        Serial.println(F("   G4 P500          ; Wait for open"));
        Serial.println(F("   M3               ; Close gripper"));
        Serial.println(F("   G4 P800          ; Wait for grip"));
        Serial.println(F("   G0 X150 Y-50 Z80 ; Move to place"));
        Serial.println(F("   M400"));
        Serial.println(F("   M5               ; Release"));
        Serial.println(F("   G4 P500"));
        Serial.println(F("   G28              ; Return home"));
        
        Serial.println(F("\n4. Gripper Test:"));
        Serial.println(F("   M6               ; Home gripper"));
        Serial.println(F("   G4 P500"));
        Serial.println(F("   M3 S5            ; Close to 5mm"));
        Serial.println(F("   G4 P1000"));
        Serial.println(F("   M3 S15           ; Close to 15mm"));
        Serial.println(F("   G4 P1000"));
        Serial.println(F("   M3               ; Fully close"));
        Serial.println(F("   G4 P1000"));
        Serial.println(F("   M5               ; Open"));
    }
    
    Serial.println(F("\n--- PARAMETERS ---"));
    Serial.println(F("  F<value>  Feedrate (deg/s or mm/s)"));
    Serial.println(F("  S<value>  Gripper position (mm)"));
    Serial.println(F("  P<value>  Delay time (ms)"));
    Serial.println(F("  X Y Z     Cartesian coordinates (mm)"));
    Serial.println(F("  T1 T2 T3  Joint angles (degrees)"));
    
    Serial.println(F("\n--- TIPS ---"));
    Serial.println(F("  • Always home (G28/M6) after power on"));
    Serial.println(F("  • Use M400 before G4 for accurate delays"));
    Serial.println(F("  • Add G4 P500 after gripper commands"));
    Serial.println(F("  • Check position with M114 anytime"));
    Serial.println(F("  • Disable motors (M18) when not in use"));
    Serial.println(F("  • All commands return 'ok' when done"));
    Serial.println(F("  • Use ';' for comments in G-code"));
    if (gripper) {
        Serial.println(F("  • Use M3001 for quick gripper check"));
    }
    
    Serial.println(F("\n--- CALIBRATION & TROUBLESHOOTING ---"));
    Serial.println(F("\n>> Motor Direction Check:"));
    Serial.println(F("1. Enable relative mode and test each axis:"));
    Serial.println(F("   G91              ; Set relative mode"));
    Serial.println(F("   G0 X10 F10       ; Move X +10mm slowly"));
    Serial.println(F("   G0 Y10 F10       ; Move Y +10mm slowly"));
    Serial.println(F("   G0 Z10 F10       ; Move Z +10mm slowly"));
    Serial.println();
    Serial.println(F("2. Expected end-effector movement:"));
    Serial.println(F("   X+10 → Forward (away from base)"));
    Serial.println(F("   Y+10 → Up (vertical)"));
    Serial.println(F("   Z+10 → Right (when facing robot)"));
    Serial.println();
    Serial.println(F("3. If direction is WRONG:"));
    Serial.println(F("   For NEMA17: Set MOTOR_X_INVERT=true in Config_Robot.h"));
    Serial.println(F("   For Gripper: Reverse wire order on ULN2003"));
    Serial.println(F("     Example: If pins are IN1→A0, IN2→A1, IN3→A2, IN4→A3"));
    Serial.println(F("              Wire them as IN1→A3, IN2→A2, IN3→A1, IN4→A0"));
    Serial.println();
    Serial.println(F("4. Return to absolute mode:"));
    Serial.println(F("   G90              ; Set absolute mode"));
    Serial.println();
    Serial.println(F(">> Steps/Degree Calibration:"));
    Serial.println(F("1. Mark current position on each joint"));
    Serial.println(F("2. G91              ; Relative mode"));
    Serial.println(F("3. G0 T190 F30      ; Move joint 1 by 90 degrees"));
    Serial.println(F("4. Measure actual rotation with protractor"));
    Serial.println(F("5. If moved X degrees instead of 90:"));
    Serial.println(F("   New_value = (90 / X) * 5.0"));
    Serial.println(F("   Update STEPS_PER_DEGREE in Config_Robot.h"));
    Serial.println();
    Serial.println(F(">> Gripper Calibration:"));
    Serial.println(F("1. M6               ; Home gripper (fully open)"));
    Serial.println(F("2. Measure jaw opening with calipers"));
    Serial.println(F("3. M3 S10           ; Command 10mm movement"));
    Serial.println(F("4. Measure new jaw opening"));
    Serial.println(F("5. If moved X mm instead of 10:"));
    Serial.println(F("   New_value = (10 / X) * 409.6"));
    Serial.println(F("   Update GRIPPER_STEPS_PER_MM in Config_Robot.h"));
    
    Serial.println(F("\n--- COMMON ISSUES ---"));
    Serial.println(F("\nMotors not moving?"));
    Serial.println(F("  → Send M17 to enable motors"));
    Serial.println(F("  → Check power supply (12V for NEMA17)"));
    Serial.println(F("  → Verify wiring matches Config_Pinout.h"));
    Serial.println();
    Serial.println(F("Position inaccurate?"));
    Serial.println(F("  → Manually position at home before power-on"));
    Serial.println(F("  → Send G28 to reset to home position"));
    Serial.println(F("  → Calibrate STEPS_PER_DEGREE (see above)"));
    Serial.println();
    Serial.println(F("Movement fails?"));
    Serial.println(F("  → Check joint limits with M114"));
    Serial.println(F("  → Position may be outside workspace"));
    Serial.println(F("  → Try intermediate positions"));
    Serial.println();
    Serial.println(F("Motor moves wrong direction?"));
    Serial.println(F("  → Set MOTOR_X_INVERT=true in Config_Robot.h"));
    Serial.println(F("  → Or swap motor wiring physically"));
    Serial.println();
    Serial.println(F("Gripper wrong direction?"));
    Serial.println(F("  → Reverse all 4 wires on ULN2003 board"));
    Serial.println(F("  → Keep Config_Pinout.h unchanged"));
    Serial.println();
    Serial.println(F("Gripper position inaccurate?"));
    Serial.println(F("  → Calibrate GRIPPER_STEPS_PER_MM (see above)"));
    Serial.println(F("  → Send M6 to re-home"));
    
    Serial.println(F("\n========================================\n"));
}