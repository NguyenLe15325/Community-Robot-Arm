Quick Setup Guide & Calibration
===============================

Flashing the Firmware
---------------------
1. Compile and upload the firmware to your Arduino board using your preferred IDE.
2. Open the Serial Monitor and establish a connection using a baud rate of ``115200``.
3. Upon a successful boot, the Serial Monitor should output the system status as ``READY``.

Validating Motor Direction
--------------------------
1. With the power off, manually articulate the robot arm into its home position.

   .. image:: ../assets/home.jpg
      :alt: Home pose and xyz definition
      :align: center

2. Energize the motors by sending the ``M17`` command via the Serial Monitor. All three NEMA17 motors should engage and hold their current positions.
3. At the home position, the system's expected joint states are ``T1=0``, ``T2=90``, and ``T3=0``.
4. Carefully command a small movement for each joint to verify that the physical rotation aligns with the firmware's defined positive direction:

   .. image:: ../assets/sideview.jpg
      :alt: Side view with theta2/theta3 sign convention
      :align: center

5. You can test joint movements by utilizing standard G1 commands. For instance, executing ``G1 T145 T280 T310`` will command Theta1 to 45°, Theta2 to 80°, and Theta3 to 10°.
   
   *Note:* The relative positioning mode (``G91``) is strictly for Cartesian movements. Joint angle commands (T1, T2, T3) require absolute mode to function correctly.

6. For isolated testing, you may command each axis individually:
   
   * ``G1 T145`` (Moves Theta1 to 45°)
   * ``G1 T280`` (Moves Theta2 to 80°)
   * ``G1 T310`` (Moves Theta3 to 10°)

7. Should any axis rotate opposite to the intended direction:
   
   * Invert the corresponding axis by toggling the ``MOTORx_INVERT`` parameter in ``Config_Robot.h``, or physically reverse the motor wiring and reflash the firmware.
   * If the gripper mechanism's open/close logic is reversed, set ``GRIPPER_INVERT_DIRECTION`` to ``true``.

Testing the Endstops
--------------------
**Critical Warning:** It is imperative to verify that all endstops are wired correctly and triggering properly prior to executing a homing sequence. Failure to do so may result in mechanical damage.

1. Send the ``M119`` command to poll the current status of all endstops.
2. Based on the default active-low logic, the endstops should register as LOW when unpressed, and HIGH when triggered.
3. Manually depress an endstop and send ``M119`` again. For instance, depressing the Theta1 endstop should yield the output: ``ENDSTOPS: 1 0 0``.

Executing the Homing Sequence
-----------------------------
1. Initiate the homing procedure by sending the ``G28`` command.
2. The robot will systematically seek its endstops. Upon completion, the arm will rest at the defined home position.
3. The robot arm is now calibrated and ready for standard operation.

Emergency Stop Procedures
-------------------------
1. To trigger a software emergency stop, send the ``!`` character or the ``M112`` command.
2. This immediately halts all ongoing motion and de-energizes the stepper motors.
3. Once an emergency stop has been engaged, the robot must be re-homed before resuming normal operation.
4. *Safety Note:* While software stops are useful, the most reliable emergency stop method is mechanically disconnecting the main power supply to the Arduino.

Calibration Guidelines
----------------------

**Verifying Steps Per Degree**
The ``STEPS_PER_DEGREE`` parameter dictates positional accuracy. This value must encapsulate the motor's native step count, the driver's microstepping configuration, and any mechanical gear reduction.

Formula: ``steps_per_degree = (motor_steps_per_rev * microstepping * gear_ratio) / 360``

**Tuning Home Offsets**
The variables ``HOME_OFFSET_STEPS_THETA*`` represent signed step movements applied immediately after an endstop is triggered during homing.

To precisely tune these offsets:
1. Execute a homing sequence (``G28``).
2. Query the current mechanical pose using ``M114``.
3. Compare the measured pose against your desired ``HOME_THETA*`` values.
4. Adjust the offset parameters in the firmware configuration accordingly, and reflash the board.
