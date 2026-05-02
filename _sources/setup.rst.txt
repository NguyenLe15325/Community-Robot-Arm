Commissioning & Calibration
===========================

Initial Deployment
------------------
1. Compile the firmware utilizing the Arduino IDE or ``arduino-cli`` and flash the compiled binary to the microcontroller.
2. Initialize a serial connection to the board at a baud rate of ``115200``.
3. Confirm system initialization by verifying the ``READY`` broadcast in the serial console.

Kinematic Direction Validation
------------------------------
Before commanding automated movements, the physical rotation of each axis must be verified against the firmware's kinematic model.

1. Disconnect motor power and manually manipulate the arm into the mechanical home orientation:

   .. image:: ../assets/home.jpg
      :alt: Mechanical Home Orientation
      :align: center

2. Transmit the ``M17`` command to energize the drivers. The system will lock the joints in place.
3. In this posture, the firmware assumes the internal joint angles are: **Theta 1 = 0°**, **Theta 2 = 90°**, and **Theta 3 = 0°**.
4. Reference the coordinate diagram below to understand the defined positive rotation vectors:

   .. image:: ../assets/sideview.jpg
      :alt: Kinematic Sign Convention
      :align: center

5. Command isolated, low-displacement movements to each joint using absolute positioning commands (e.g., ``G1 T145`` to drive Theta 1 to 45°). Note that incremental positioning (``G91``) is invalid for direct joint-space (T-parameter) commands.
6. If an actuator rotates counter to the mathematical model:
   * **Software Resolution**: Invert the logic by toggling the respective ``MOTORx_INVERT`` boolean in ``Config_Robot.h`` and recompile.
   * **Hardware Resolution**: De-energize the system and reverse the physical coil wiring for the affected stepper motor.

Endstop Diagnostics
-------------------
**Caution:** Executing a homing cycle with malfunctioning limit switches will result in mechanical collisions and potential hardware failure.

1. Issue the ``M119`` command to request an endstop telemetry report.
2. Verify that all switches reflect an open state (``0`` for active-low configurations).
3. Manually actuate each limit switch sequentially while polling ``M119`` to confirm the state transitions to triggered (``1``).

Automated Homing Sequence
-------------------------
1. Transmit the ``G28`` command to initiate the zeroing protocol.
2. The controller will systematically drive all axes toward their respective limit switches.
3. Upon switch contact, the system applies predefined offset steps and sets the internal kinematic state to the configured home angles.

Emergency Intervention
----------------------
In the event of anomalous behavior or impending mechanical collision:
* Transmit the ``M112`` command (or the ``!`` alias) to trigger an immediate software halt. This routine bypasses the motion planner and immediately drops the driver ENABLE line.
* Following a software halt, the kinematic state is considered invalid; a complete ``G28`` homing cycle is required before operations can resume.
* **Hardware Override**: Physically severing the primary DC power supply remains the most robust method for emergency intervention.

System Calibration
------------------

**Resolution Tuning (Steps Per Degree)**
The ``STEPS_PER_DEGREE`` constant defines the relationship between motor pulses and physical displacement. It must be precisely calculated based on the motor's step angle, the driver's microstepping multiplier, and the mechanical gear reduction.

.. math::
   \text{Steps}/\text{Degree} = \frac{\text{Motor Steps Per Revolution} \times \text{Microstepping Factor} \times \text{Gear Ratio}}{360}

**Home Offset Calibration**
The parameters denoted as ``HOME_OFFSET_STEPS_THETA*`` govern the signed step displacement applied after the limit switch is triggered.

To calibrate the mechanical origin:
1. Execute a full homing cycle (``G28``).
2. Query the current logical position via ``M114``.
3. Measure the actual physical angles of the robot.
4. Calculate the discrepancy between the logical and physical states, convert this delta into steps, and update the offset variables within the firmware configuration.
