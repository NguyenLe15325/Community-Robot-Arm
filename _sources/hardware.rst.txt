Hardware Setup
==============

What You Need
-------------
The Community Robot Arm uses standard, easy-to-find parts. For the electronics, you will need:

* **Main Motors**: Three NEMA17 Stepper Motors, with standard drivers (A4988 or DRV8825).
* **Gripper Motor**: One BYJ-48 Stepper Motor and a ULN2003 driver board for the gripper.
* **Microcontroller**: Arduino Uno or Arduino Nano.
* **Expansion Shield**: CNC GRBL Shield (Version 3 for Uno; Version 4 for Nano).
* **Limit Switches**: Three mechanical switches for homing the arm.
* **Power Supply**: A DC power supply that can power all four motors at once.
* **Safety**: A shared ENABLE wire connected to all NEMA drivers so you can quickly cut power if needed.

3D Printed Parts
----------------
The arm's structure is based on the open-source Florin Tobler design. You can download the STL files to 3D print the parts here: `Community Robot Arm Repository <https://grabcad.com/library/robot-arm-community-version-cad-3d-printed-robotic-arm-1>`_.

Important Hardware Notes
------------------------

**1. Setting Motor Current (VREF)**
Before connecting the NEMA17 motors, you must manually tune the reference voltage (VREF) on each A4988 or DRV8825 driver using the small screw (potentiometer). This limits the maximum current going to the motors. If you skip this, your drivers will overheat and break. For step-by-step instructions, see these guides:

* `DRV8825 Stepper Motor Driver Tutorial <https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/>`_
* `A4988 Stepper Motor Driver Tutorial <https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/>`_

**2. Fixing the CNC V4 Shield Bug**
If you are using an Arduino Nano with the CNC V4 Shield, you need to fix a known manufacturing defect on the board. The microstepping pins (MS1, MS2, MS3) are accidentally connected to ground by the board's copper traces, forcing the motors to run in noisy full-step mode. To fix this and get smoother movement, you must cut those traces and connect the pins to 5V. Read this guide for the fix: `How to Use the CNC V4 Board despite Its quirks <https://www.instructables.com/How-to-Use-the-CNC-V4-Board-despite-Its-quirks/>`_.

Wiring Guide
------------
The software expects specific pins to be used. This mapping is defined in ``firmware/RobotArm/Config_Pinout.h``. Make sure you wire everything exactly like this:

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

Wiring Limit Switches
---------------------
The software uses the Arduino's built-in pull-up resistors (``INPUT_PULLUP``). By default, it expects the switches to trigger when connected to ground (``ENDSTOP_ACTIVE_LOW true``). 

To wire the limit switches for the default setup:

1. Connect the Common (COM) pin of the switch to Ground.
2. Connect the Normally Open (NO) pin of the switch to the correct Arduino signal pin.

With this setup, the pin reads HIGH when the switch is untouched, and LOW when it is pressed. If your setup requires the opposite, change ``ENDSTOP_ACTIVE_LOW`` to false in the code before uploading.
