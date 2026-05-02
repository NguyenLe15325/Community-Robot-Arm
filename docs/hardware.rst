Hardware & Electrical Specifications
====================================

System Requirements
-------------------
The Community Robot Arm relies on standard, accessible components. To assemble the electronics package, you will need the following:

* **Primary Actuators**: Three NEMA17 Stepper Motors, controlled by standard step/direction drivers (A4988 or DRV8825).
* **End Effector Actuator**: One BYJ-48 Stepper Motor, driven by a ULN2003 control board, dedicated to the gripper mechanism.
* **Microcontroller Unit (MCU)**: Arduino Uno or Arduino Nano.
* **Expansion Shield**: CNC GRBL Shield (Version 3 for the Uno architecture; Version 4 for the Nano architecture).
* **Limit Switches**: Three mechanical switches for establishing the kinematic home position.
* **Power Supply**: A DC power supply capable of sustaining the peak current draw of all four motors simultaneously.
* **Safety Hardware**: A shared hardware ENABLE line routed to all NEMA drivers to allow immediate de-energization.

Structural Components
---------------------
The kinematic structure follows the open-source Florin Tobler architecture. The required STL geometry files for additive manufacturing are available via the `Community Robot Arm Repository <https://grabcad.com/library/robot-arm-community-version-cad-3d-printed-robotic-arm-1>`_.

Critical Hardware Advisories
----------------------------

**1. Stepper Driver Current Calibration**
Prior to connecting the NEMA17 motors, the reference voltage (VREF) on each A4988 or DRV8825 driver must be manually tuned using the onboard potentiometer. This limits the maximum current delivered to the motor coils. Failure to perform this calibration will result in severe thermal overload and permanent driver degradation. For detailed step-by-step calibration instructions, please refer to the following documentation:
* `DRV8825 Stepper Motor Driver Tutorial <https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/>`_
* `A4988 Stepper Motor Driver Tutorial <https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/>`_

**2. GRBLv4 Shield Microstepping Defect**
System integrators utilizing the Arduino Nano paired with the CNC V4 Shield must address a documented PCB manufacturing defect. The microstepping selection pins (MS1, MS2, MS3) are permanently grounded by the board's copper traces, restricting the system to full-step mode. To enable microstepping for smoother motion profiles, the grounded traces must be severed and the pins tied to a 5V logic source. For a comprehensive walkthrough of this remediation, refer to the guide: `How to Use the CNC V4 Board despite Its quirks <https://www.instructables.com/How-to-Use-the-CNC-V4-Board-despite-Its-quirks/>`_.

Wiring Architecture
-------------------
The firmware relies on a static pin mapping defined within ``firmware/RobotArm/Config_Pinout.h``. Ensure all physical connections strictly adhere to the following routing table:

+-----------------------------+--------------+
| Component Function          | Arduino Pin  |
+=============================+==============+
| Theta 1 (Base) Step         | D6           |
+-----------------------------+--------------+
| Theta 1 (Base) Direction    | D3           |
+-----------------------------+--------------+
| Theta 2 (Shoulder) Step     | D5           |
+-----------------------------+--------------+
| Theta 2 (Shoulder) Direction| D2           |
+-----------------------------+--------------+
| Theta 3 (Elbow) Step        | D7           |
+-----------------------------+--------------+
| Theta 3 (Elbow) Direction   | D4           |
+-----------------------------+--------------+
| Global Driver Enable        | D8           |
+-----------------------------+--------------+
| Theta 1 Limit Switch        | D10          |
+-----------------------------+--------------+
| Theta 2 Limit Switch        | D9           |
+-----------------------------+--------------+
| Theta 3 Limit Switch        | D11          |
+-----------------------------+--------------+
| Gripper IN1                 | A0           |
+-----------------------------+--------------+
| Gripper IN2                 | A1           |
+-----------------------------+--------------+
| Gripper IN3                 | A2           |
+-----------------------------+--------------+
| Gripper IN4                 | A3           |
+-----------------------------+--------------+

Endstop Integration
-------------------
The firmware utilizes the internal ``INPUT_PULLUP`` resistors on the microcontroller. By default, the system anticipates an active-low triggering mechanism (defined by ``ENDSTOP_ACTIVE_LOW true``). 

To wire the endstops under the default configuration:
1. Connect the Common (COM) terminal of the microswitch to Ground.
2. Connect the Normally Open (NO) terminal to the designated signal pin.

Under this topology, an idle switch registers a logic HIGH, while a physically depressed switch pulls the logic LOW, signaling a collision or home state. If your electrical architecture demands an active-high configuration, update the ``ENDSTOP_ACTIVE_LOW`` macro in the firmware configuration header before compilation.
