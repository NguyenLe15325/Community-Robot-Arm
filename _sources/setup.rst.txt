Quick Setup Guide & Calibration
===============================

Flash Firmware to Arduino
-------------------------
1. Upload firmware to the Arduino board.
2. Use the serial monitor to check if the firmware is uploaded successfully (Baud rate: ``115200``).
3. After upload, the serial monitor should show ``READY``.

Test Motor Direction
--------------------
1. Manually place your robot at its home position.

   .. image:: ../assets/home.jpg
      :alt: Home pose and xyz definition
      :align: center

2. Enable motors by sending ``M17``. All NEMA17 motors should be energized and hold their position.
3. At the home position, the expected state is ``T1=0``, ``T2=90``, ``T3=0``.
4. Try moving each theta by a small angle to check if the direction is correct. Positive rotation directions are defined as follows:

   .. image:: ../assets/sideview.jpg
      :alt: Side view with theta2/theta3 sign convention
      :align: center

5. Move individual joints using G1 commands (e.g., ``G1 T145 T280 T310`` to move theta1 to 45 degree, theta2 to 80 degree, theta3 to 10 degree).
   
   *Note:* Relative mode ``G91`` doesn't apply to T1, T2, T3 commands (forward kinematics commands) - only absolute mode.

6. Alternatively, you can run each theta move command separately:
   
   * ``G1 T145`` (move theta1 to 45 degree)
   * ``G1 T280`` (move theta2 to 80 degree)
   * ``G1 T310`` (move theta3 to 10 degree)

7. Toggle direction in the firmware (edit ``Config_Robot.h`` file) if needed, or hard wire and reflash:
   
   * If a NEMA axis moves opposite to expected, flip ``MOTORx_INVERT``.
   * If gripper close/open is reversed, set ``GRIPPER_INVERT_DIRECTION``.

Test Endstops
-------------
**Critical:** Make sure the endstop is wired correctly and working before homing.

1. Command to check endstop status: ``M119``
2. Endstops should be LOW when not pressed and HIGH when pressed (or according to your logic config).
3. For example, when you press the theta1 endstop, the output should be: ``ENDSTOPS: 1 0 0``

Home Robot
----------
1. Command to home robot: ``G28``
2. After homing, the robot should be at its home position.
3. Now the robot is ready to use.

Emergency Stop
--------------
1. Command to emergency stop: ``!`` or ``M112``
2. After an emergency stop, the robot will stop immediately and disable all motors.
3. Rehome the robot to use it again.
4. *Note:* Disconnecting power is the most reliable way to emergency stop the robot; simply disconnect the Arduino power supply.

Calibration Guide
-----------------
**Verify steps per degree**
``STEPS_PER_DEGREE`` should include full-step count, microstep setting, and mechanical ratio.

Formula: ``steps_per_degree = (motor_steps_per_rev * microstepping * gear_ratio) / 360``

**Tune home offsets**
``HOME_OFFSET_STEPS_THETA*`` are signed step moves applied after the endstop hit.

Workflow:
1. Run ``G28``
2. Read ``M114``
3. Compare measured pose against desired ``HOME_THETA*``
4. Adjust offsets in firmware and reflash.
