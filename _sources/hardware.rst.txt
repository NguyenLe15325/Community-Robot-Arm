Hardware Requirements & Wiring
==============================

Required Hardware
-----------------
To successfully assemble and operate the Community Robot Arm, the following hardware components are required:

* **NEMA17 Stepper Motors (x3)**: Required for the main joints. Pair these with either DRV8825 or A4988 stepper drivers.
   * *Important:* You must manually calibrate the current limit for each driver using the onboard VR1 potentiometer. For detailed instructions, refer to these tutorials:
      * `Tuning the DRV8825 <https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/>`_
      * `Tuning the A4988 <https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/>`_
* **BYJ-48 Stepper Motor (x1)**: Utilized alongside its corresponding driver to operate the gripper mechanism.
* **Microcontroller**: An Arduino Uno or Arduino Nano.
* **Arduino GRBL Shield**: Use GRBLv3 for the Arduino Uno, or GRBLv4 for the Arduino Nano.
   * *Hardware Notice:* The GRBLv4 shield designed for the Arduino Nano contains a known design flaw regarding microstepping pin selection. By default, the microstepping pins from the drivers (A4988 or DRV8825) are pulled LOW. They must be set HIGH to configure microstepping properly, but the GRBLv4 shield grounds the MS select trace instead of providing 5V. For a comprehensive workaround, please consult this guide: `Resolving CNC V4 Board Quirks <https://www.instructables.com/How-to-Use-the-CNC-V4-Board-despite-Its-quirks/>`_.
* **Limit Switches (x3)**: Used for homing and endstop detection.
* **Shared Enable Line**: Must be wired to all NEMA drivers.
* **Power Supply**: Ensure your power supply is adequately rated to handle the simultaneous draw of all motors and drivers.

3D Printed Components
---------------------
The mechanical structure is based on the Florin Tobler robot arm design. The necessary STL files for 3D printing can be downloaded here: `Community Robot Arm CAD Files <https://grabcad.com/library/robot-arm-community-version-cad-3d-printed-robotic-arm-1>`_.

Wiring & Pinout Configuration
-----------------------------
Wiring connections must accurately follow the pinout configuration defined in ``firmware/RobotArm/Config_Pinout.h``. 

+--------------------------+-----+
| Function                 | Pin |
+==========================+=====+
| Theta1 STEP              | D6  |
+--------------------------+-----+
| Theta1 DIR               | D3  |
+--------------------------+-----+
| Theta2 STEP              | D5  |
+--------------------------+-----+
| Theta2 DIR               | D2  |
+--------------------------+-----+
| Theta3 STEP              | D7  |
+--------------------------+-----+
| Theta3 DIR               | D4  |
+--------------------------+-----+
| Shared EN (NEMA drivers) | D8  |
+--------------------------+-----+
| Theta1 endstop           | D10 |
+--------------------------+-----+
| Theta2 endstop           | D9  |
+--------------------------+-----+
| Theta3 endstop           | D11 |
+--------------------------+-----+
| Gripper IN1              | A0  |
+--------------------------+-----+
| Gripper IN2              | A1  |
+--------------------------+-----+
| Gripper IN3              | A2  |
+--------------------------+-----+
| Gripper IN4              | A3  |
+--------------------------+-----+

Please ensure that you connect each motor to its corresponding joint definitions. Additionally, connect the gripper and endstops accordingly. If your physical wiring differs from the defaults, be sure to update the configuration file before compiling the firmware.

Endstop Logic and Wiring
------------------------
* The endstop pins are internally configured using the ``INPUT_PULLUP`` state.
* By default, the firmware assumes an active-low triggered logic (``ENDSTOP_ACTIVE_LOW true``).
* For standard operation using the default settings, wire the endstops as follows:
   * Connect one terminal of each switch to Ground (GND).
   * Connect the other terminal to the respective endstop pin.
* With this wiring configuration:
   * An open switch registers as HIGH (Not Triggered).
   * A pressed switch registers as LOW (Triggered).

*Note:* If you are utilizing active-high hardware, you must modify the ``ENDSTOP_ACTIVE_LOW`` flag in ``Config_Robot.h`` to reflect this physical change.
