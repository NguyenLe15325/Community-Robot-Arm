Serial Protocol & Command Reference
===================================

Serial Protocol
---------------
The software uses a simple text-based serial protocol to communicate. 

* **Baud Rate**: ``115200`` (default)
* **Line Ending**: Newline or carriage return
* **Startup**: Prints ``READY`` when the board boots up
* **Success**: Prints ``ok`` when a command finishes
* **Errors**: Prints ``Error: <message>`` if you send a bad command
* **Emergency**: Prints ``ALARM:ESTOP`` if an emergency stop happens

Supported Commands
------------------

Motion and Modes
^^^^^^^^^^^^^^^^
+-------------------+---------------------------------------------------+
| Command           | Description                                       |
+===================+===================================================+
| ``G0`` / ``G1``   | Linear move command (Joint-space or Cartesian)    |
+-------------------+---------------------------------------------------+
| ``G4 P<ms>``      | Dwell command (utilizes non-blocking wait loops)  |
+-------------------+---------------------------------------------------+
| ``G28 [F<deg/s>]``| Initiate homing sequence at specified speed       |
+-------------------+---------------------------------------------------+
| ``G90``           | Set absolute positioning mode (Cartesian)         |
+-------------------+---------------------------------------------------+
| ``G91``           | Set relative positioning mode (Cartesian)         |
+-------------------+---------------------------------------------------+

**G0/G1 Parameters:**

* **Joint Moves**: ``T1<deg>``, ``T2<deg>``, ``T3<deg>``
* **Cartesian Moves**: ``X``, ``Y``, ``Z`` (in millimeters)
* **Speed**: ``F<deg/s>`` (Controls how fast the main motors turn)

Motor Power and Diagnostics
^^^^^^^^^^^^^^^^^^^^^^^^^^^
+-------------------------------+----------------------------------------------------+
| Command                       | Description                                        |
+===============================+====================================================+
| ``M17``                       | Energize all arm stepper drivers                   |
+-------------------------------+----------------------------------------------------+
| ``M18`` / ``M84``             | De-energize all arm stepper drivers                |
+-------------------------------+----------------------------------------------------+
| ``M114``                      | Query current Cartesian and joint-space state      |
+-------------------------------+----------------------------------------------------+
| ``M119``                      | Query the active logic states of all endstops      |
+-------------------------------+----------------------------------------------------+
| ``M205 [S<ramp>] [F<min>]``   | Query or modify the motion smoothing parameters    |
+-------------------------------+----------------------------------------------------+
| ``M400``                      | Block serial processing until motion queue empties |
+-------------------------------+----------------------------------------------------+
| ``HELP``                      | Display a compact command reference table          |
+-------------------------------+----------------------------------------------------+

Gripper Control
^^^^^^^^^^^^^^^
+------------------------+-------------------------------------------------------------+
| Command                | Description                                                 |
+========================+=============================================================+
| ``M3 [F<mm/s>]``       | Actuate gripper close (uses default relative distance)      |
+------------------------+-------------------------------------------------------------+
| ``M3 S<mm> [F<mm/s>]`` | Actuate gripper close by ``S`` mm (relative, positive = close)|
+------------------------+-------------------------------------------------------------+
| ``M5 [F<mm/s>]``       | Actuate gripper open (uses default relative distance)       |
+------------------------+-------------------------------------------------------------+
| ``M5 S<mm> [F<mm/s>]`` | Actuate gripper open by ``S`` mm (relative, positive = open)|
+------------------------+-------------------------------------------------------------+
| ``M6 [F<mm/s>]``       | Home the gripper mechanism (performs relative limit test)   |
+------------------------+-------------------------------------------------------------+
| ``M3001``              | Query the current status of the gripper mechanism           |
+------------------------+-------------------------------------------------------------+

Emergency Halts
^^^^^^^^^^^^^^^
+-----------+-----------------------------------+
| Command   | Description                       |
+===========+===================================+
| ``M112``  | Immediate software emergency stop |
+-----------+-----------------------------------+
| ``!``     | Instantaneous alias for ``M112``  |
+-----------+-----------------------------------+

*Note:* Sending the Ctrl-X (``0x18``) byte over serial will instantly trigger an emergency stop, even if the robot is busy doing something else.
