#include "GCode.h"
#include "Config_Robot.h"

// --- Initialization ---
void GCodeParser::begin(NEMA17Controller* controller, BYJ48Gripper* gripperController) {
    motor = controller;
    gripper = gripperController;
    verboseMode = false;
    inputBuffer = "";
    inputBuffer.reserve(128);
    absoluteMode = true;
    currentFeedrate = 60.0; // Default 60 deg/s (joint-space speed)
    
    // Initialize delay state
    delaying = false;
    delayStartTime = 0;
    delayDuration = 0;
}

// --- Update Loop ---
void GCodeParser::update() {
    // Check if we're in a non-blocking delay
    if (delaying) {
        if (motor->serviceEmergencyStopInput()) {
            motor->consumeEmergencyStopLatch();
            delaying = false;
            if (gripper) {
                gripper->stop();
                gripper->disable();
            }
            Serial.println(F("ALARM:ESTOP"));
            sendError("Delay aborted");
            return;
        }

        if (millis() - delayStartTime >= delayDuration) {
            delaying = false;
            debugPrintln(F("G4 complete"));
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
void GCodeParser::sendError(const String& error) {
    Serial.print(F("Error: "));
    Serial.println(error);
}

void GCodeParser::sendOK() {
    Serial.println(F("ok"));
}

void GCodeParser::debugPrintPrefix() {
    Serial.print(F("[DBG] "));
}

void GCodeParser::debugPrintln(const __FlashStringHelper* message) {
    if (!verboseMode) {
        return;
    }
    debugPrintPrefix();
    Serial.println(message);
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
    } else if (processedLine == "!") {
        // Optional quick emergency stop alias
        cmd.command = 'M';
        cmd.number = 112;
        return true;
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
            case 112: return handleM112(cmd);
            case 114: return handleM114(cmd);
            case 119: return handleM119(cmd);
            case 205: return handleM205(cmd);
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
            debugPrintPrefix();
            Serial.print(F("G0/G1 joint target [deg] T1="));
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
            debugPrintPrefix();
            Serial.print(F("G0/G1 cart target [mm] X="));
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
        debugPrintPrefix();
        Serial.print(F("G4 start P="));
        Serial.print(delayDuration);
        Serial.println(F("ms"));
    }
    
    delaying = true;
    delayStartTime = millis();
    
    // Don't send OK yet - will send when delay completes
    return true;
}

bool GCodeParser::handleG28(const GCodeCommand& cmd) {
    // Home the robot
    debugPrintln(F("G28 start"));

    float homingFeedrate = HOMING_SEEK_FEEDRATE;
    if (cmd.hasF) {
        homingFeedrate = cmd.f;
    }
    
    if (!motor->home(homingFeedrate)) {
        if (motor->consumeEmergencyStopLatch()) {
            if (gripper) {
                gripper->stop();
                gripper->disable();
            }
            Serial.println(F("ALARM:ESTOP"));
            sendError("Homing aborted");
        } else {
            sendError("Homing failed");
        }
        return false;
    }
    
    // Wait for homing to complete
    while (motor->isMoving()) {
        motor->update();
        if (motor->serviceEmergencyStopInput()) {
            motor->consumeEmergencyStopLatch();
            if (gripper) {
                gripper->stop();
                gripper->disable();
            }
            Serial.println(F("ALARM:ESTOP"));
            sendError("Homing aborted");
            return false;
        }
        delay(1);
    }
    
    if (gripper) {
        while (gripper->isMoving()) {
            gripper->update();
            if (motor->serviceEmergencyStopInput()) {
                motor->consumeEmergencyStopLatch();
                gripper->stop();
                gripper->disable();
                Serial.println(F("ALARM:ESTOP"));
                sendError("Homing aborted");
                return false;
            }
            delay(1);
        }
    }
    
    debugPrintln(F("G28 done"));
    
    return true;
}

bool GCodeParser::handleG90(const GCodeCommand& cmd) {
    absoluteMode = true;
    debugPrintln(F("Mode set: G90 absolute"));
    return true;
}

bool GCodeParser::handleG91(const GCodeCommand& cmd) {
    absoluteMode = false;
    debugPrintln(F("Mode set: G91 relative"));
    return true;
}

bool GCodeParser::handleM17(const GCodeCommand& cmd) {
    motor->enable();
    debugPrintln(F("M17 motors enabled"));
    return true;
}

bool GCodeParser::handleM18(const GCodeCommand& cmd) {
    motor->disable();
    if (gripper) gripper->disable();
    debugPrintln(F("M18/M84 motors disabled"));
    return true;
}

bool GCodeParser::handleM84(const GCodeCommand& cmd) {
    return handleM18(cmd); // Alias for M18
}

bool GCodeParser::handleM112(const GCodeCommand& cmd) {
    delaying = false;
    motor->emergencyStop();
    motor->disable();
    if (gripper) {
        gripper->stop();
        gripper->disable();
    }
    Serial.println(F("ALARM:ESTOP"));
    debugPrintln(F("M112 emergency stop"));
    // Intentionally suppress normal "ok" after emergency stop.
    return false;
}

bool GCodeParser::handleM3(const GCodeCommand& cmd) {
    // M3: Close gripper or move to position
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    float speedMmPerSec = GRIPPER_DEFAULT_SPEED;
    if (cmd.hasF) speedMmPerSec = cmd.f;
    float speedStepsPerSec = gripper->mmPerSecToStepsPerSec(speedMmPerSec);
    
    if (cmd.hasS) {
        // Move to specific position
        if (verboseMode) {
            debugPrintPrefix();
            Serial.print(F("M3 setpoint [mm] S="));
            Serial.print(cmd.s);
            Serial.print(F(" F(mm/s)="));
            Serial.println(speedMmPerSec, 2);
        }
        gripper->moveToPosition(cmd.s, speedStepsPerSec);
    } else {
        // Close gripper fully
        if (verboseMode) {
            debugPrintPrefix();
            Serial.print(F("M3 close F(mm/s)="));
            Serial.println(speedMmPerSec, 2);
        }
        gripper->close(speedStepsPerSec);
    }
    
    return true;
}

bool GCodeParser::handleM5(const GCodeCommand& cmd) {
    // M5: Open gripper
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    float speedMmPerSec = GRIPPER_DEFAULT_SPEED;
    if (cmd.hasF) speedMmPerSec = cmd.f;
    float speedStepsPerSec = gripper->mmPerSecToStepsPerSec(speedMmPerSec);
    
    if (verboseMode) {
        debugPrintPrefix();
        Serial.print(F("M5 open F(mm/s)="));
        Serial.println(speedMmPerSec, 2);
    }
    gripper->open(speedStepsPerSec);
    
    return true;
}

bool GCodeParser::handleM6(const GCodeCommand& cmd) {
    // M6: Home gripper
    if (!gripper) {
        sendError("Gripper not configured");
        return false;
    }
    
    float speedMmPerSec = GRIPPER_HOMING_SPEED;
    if (cmd.hasF) speedMmPerSec = cmd.f;
    float speedStepsPerSec = gripper->mmPerSecToStepsPerSec(speedMmPerSec);
    
    if (verboseMode) {
        debugPrintPrefix();
        Serial.print(F("M6 start F(mm/s)="));
        Serial.println(speedMmPerSec, 2);
    }
    
    gripper->home(speedStepsPerSec);
    
    // Wait for homing to complete
    while (gripper->isMoving()) {
        gripper->update();
        if (motor->serviceEmergencyStopInput()) {
            motor->consumeEmergencyStopLatch();
            gripper->stop();
            gripper->disable();
            Serial.println(F("ALARM:ESTOP"));
            sendError("Gripper homing aborted");
            return false;
        }
        delay(1);
    }
    
    // Set current position as zero
    gripper->setZero();
    
    debugPrintln(F("M6 done (gripper zeroed)"));
    
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

bool GCodeParser::handleM119(const GCodeCommand& cmd) {
    #if ENDSTOPS_INSTALLED
    Serial.print(F("T1:"));
    Serial.print(motor->getEndstopTriggered(0) ? F("TRIG") : F("OPEN"));
    Serial.print(F(" T2:"));
    Serial.print(motor->getEndstopTriggered(1) ? F("TRIG") : F("OPEN"));
    Serial.print(F(" T3:"));
    Serial.println(motor->getEndstopTriggered(2) ? F("TRIG") : F("OPEN"));
    #else
    Serial.println(F("ENDSTOPS:DISABLED"));
    #endif
    return true;
}

bool GCodeParser::handleM205(const GCodeCommand& cmd) {
    // M205 S<rampPortion> F<minSpeedScale>
    // - no params: query current smoothing
    // - with S/F: set one or both values (clamped by motor controller)
    float rampPortion = 0.0f;
    float minSpeedScale = 0.0f;
    motor->getMotionSmoothing(rampPortion, minSpeedScale);

    if (cmd.hasS || cmd.hasF) {
        if (cmd.hasS) {
            rampPortion = cmd.s;
        }
        if (cmd.hasF) {
            minSpeedScale = cmd.f;
        }
        motor->setMotionSmoothing(rampPortion, minSpeedScale);
        motor->getMotionSmoothing(rampPortion, minSpeedScale);
    }

    Serial.print(F("SMOOTH:RAMP="));
    Serial.print(rampPortion, 3);
    Serial.print(F(" MIN="));
    Serial.println(minSpeedScale, 3);
    return true;
}

bool GCodeParser::handleM400(const GCodeCommand& cmd) {
    // Wait for all moves to finish
    debugPrintln(F("M400 wait start"));
    
    while (motor->isMoving()) {
        motor->update();
        if (motor->serviceEmergencyStopInput()) {
            motor->consumeEmergencyStopLatch();
            if (gripper) {
                gripper->stop();
                gripper->disable();
            }
            Serial.println(F("ALARM:ESTOP"));
            sendError("Wait aborted");
            return false;
        }
        delay(1);
    }
    
    debugPrintln(F("M400 wait done"));
    
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
    Serial.println(F("CMD: G0/G1 X..Y..Z.. F | T1..T2..T3.. F"));
    Serial.println(F("CMD: G4 P | G28 [F] | G90/G91"));
    Serial.println(F("CMD: M17 M18/M84 M112 M114 M119 M205 M400 HELP"));
    if (gripper) {
        Serial.println(F("CMD: M3 [S<mm>] [F<mm/s>] M5 [F<mm/s>] M6 [F<mm/s>] M3001"));
    }
}