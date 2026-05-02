Hardware Requirements & Wiring
==============================

Required Hardware
-----------------
* **NEMA17 x3 motor** with 3 drivers (DRV8825 or A4988)
   * *Note:* You have to set the current limit for the driver (DRV8825 or A4988) using the VR1 potentiometer. See documentation for details:
      * `DRV8825 <https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/>`_
      * `A4988 <https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/>`_
* **BYJ-48 stepper motor x1** with driver (for gripper)
* **Arduino Uno or Nano x1**
* **Arduino GRBL shield** (GRBLv3 for Arduino Uno or GRBLv4 for Arduino Nano)
   * *Side note:* If you are using GRBLv4 with Arduino Nano, this board has a big design flaw in selecting microstep pins. Microsteps pins from the driver (A4988 or DRV8825) are set LOW by default and need to be set HIGH to enable microstep choosing, but the GRBLv4 shield MS select trace is GND instead of 5V. See a fix here: `How to Use the CNC V4 Board despite Its quirks <https://www.instructables.com/How-to-Use-the-CNC-V4-Board-despite-Its-quirks/>`_.
* **Limit switch x3**
* **Shared enable line** to all NEMA drivers
* **Power supply** sized for all motors and drivers

3D Printed Parts
----------------
Florin Tobler Robot arm parts can be found here: `GrabCAD Library <https://grabcad.com/library/robot-arm-community-version-cad-3d-printed-robotic-arm-1>`_

Wiring & Pinout
---------------
Wiring should be done according to the pinout configuration defined in ``firmware/RobotArm/Config_Pinout.h``.

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

* Connect to the right motor joint definition.
* Connect gripper and endstop, and toggle enabled in config if needed (edit endstop connection config according to your wiring).

Endstop Wiring and Logic
------------------------
* Endstops are configured as ``INPUT_PULLUP``.
* Default triggered logic is active-low (``ENDSTOP_ACTIVE_LOW true``).
* Practical wiring for default behavior:
   * One side of each switch to GND
   * Other side to its endstop pin
* With this wiring:
   * Switch open -> pin HIGH -> not triggered
   * Switch pressed -> pin LOW -> triggered

If your hardware is active-high, change ``ENDSTOP_ACTIVE_LOW`` in ``Config_Robot.h``.
