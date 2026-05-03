Setup & Calibration
===================

Initial Setup
-------------
1. Compile the firmware using the Arduino IDE or ``arduino-cli`` and upload it to your Arduino board.
2. Open a serial connection to the board with a baud rate of ``115200``.
3. Check that the board connects successfully by looking for the ``READY`` message in the serial monitor.

Checking Motor Directions
-------------------------
Before running any automated movements, make sure the physical rotation of each motor matches the software's expectations.

1. Disconnect motor power and manually move the arm into the mechanical home position:

   .. image:: ../assets/home.jpg
      :alt: Mechanical Home Orientation
      :align: center

2. Send the ``M17`` command to turn on the motors. The joints should lock in place.
3. In this position, the software assumes the internal joint angles are: **Theta 1 = 0°**, **Theta 2 = 90°**, and **Theta 3 = 0°**.
4. Reference the coordinate diagram below to understand the defined positive rotation vectors:

   .. image:: ../assets/sideview.jpg
      :alt: Kinematic Sign Convention
      :align: center

   .. image:: ../assets/pos_z.jpg
      :alt: Top View Sign Convention
      :align: center

5. Move each joint a little bit using absolute positioning commands (e.g., send ``G1 T145`` to move Theta 1 to 45°). Note that relative positioning (``G91``) does not work for direct joint commands (T-parameters).
6. If a motor turns in the wrong direction:

   * **Software Fix**: Change the direction in code by toggling the ``MOTORx_INVERT`` setting in ``Config_Robot.h`` and upload the code again.
   * **Hardware Fix**: Turn off the power and swap the coil wires for that stepper motor.

Testing Limit Switches
----------------------
**Caution:** Homing the arm with broken limit switches will cause the arm to crash and potentially break hardware.

1. Send the ``M119`` command to check the limit switch status.
2. Make sure all switches show as open (``0`` for default setups).
3. Manually press each limit switch one by one while sending ``M119`` again to confirm they change to triggered (``1``).

Auto Homing
-----------
1. Send the ``G28`` command to start homing.
2. The controller will move all joints toward their limit switches.
3. When a switch is hit, the system moves back by a set number of steps and sets the internal angles to the home position.

Emergency Stop
--------------
If something goes wrong or the arm is about to crash:

* Send the ``M112`` command (or just ``!``) to trigger an immediate software stop. This immediately cuts the ENABLE line to the motors.
* After a software stop, you must run a full ``G28`` homing cycle before moving the arm again.
* **Hardware Override**: Physically unplugging the main DC power supply is always the safest way to stop the robot.

Calibration
-----------

**Steps Per Degree**
The ``STEPS_PER_DEGREE`` setting tells the software how many motor steps equal one degree of movement. It needs to be calculated based on your motor, microstepping, and gear ratio.

.. math::
   \text{Steps}/\text{Degree} = \frac{\text{Motor Steps Per Revolution} \times \text{Microstepping Factor} \times \text{Gear Ratio}}{360}

**Home Offset Calibration**
The ``HOME_OFFSET_STEPS_THETA*`` settings control how many steps the arm moves away from the limit switch after hitting it.

To calibrate the home position:

1. Run a full homing cycle (``G28``).
2. Check the current software position with ``M114``.
3. Measure the actual physical angles of the robot.
4. Calculate the difference between the software angles and physical angles, convert this difference into steps, and update the offset numbers in the code.
