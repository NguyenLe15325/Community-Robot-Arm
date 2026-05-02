Kinematics & Workspace
======================

Notation and Units
------------------
+------------------------+---------------------------------------+---------------------------------------+
| Symbol                 | Meaning                               | Unit                                  |
+========================+=======================================+=======================================+
| x, y, z                | Cartesian position                    | mm                                    |
+------------------------+---------------------------------------+---------------------------------------+
| L                      | Effective link length                 | mm                                    |
+------------------------+---------------------------------------+---------------------------------------+
| a                      | Base offset                           | mm                                    |
+------------------------+---------------------------------------+---------------------------------------+
| theta1, theta2, theta3 | Joint angles                          | deg (docs), rad (computation)         |
+------------------------+---------------------------------------+---------------------------------------+

Mechanical Model
----------------
The physical arm uses shoulder/elbow 4-bar linkages. For kinematic computation, firmware uses an equivalent 3-DOF model:

* Base rotation joint: ``theta1``
* Shoulder-equivalent joint: ``theta2``
* Elbow-equivalent joint: ``theta3``
* Model constants: ``L = 140.0 mm``, ``a = 54.0 mm``

Home Pose and Axis Frame
------------------------
* **theta1** = 0 deg (Robot facing middle/front)
* **theta2** = 90 deg (Lower shank perpendicular to ground plane)
* **theta3** = 0 deg (Upper shank parallel to ground plane)

.. image:: ../assets/home.jpg
   :alt: Home pose and xyz definition
   :align: center

Positive/Negative Rotation Directions
-------------------------------------

**Base axis (theta1)**
Base rotation sign follows the base/top-view convention.

.. image:: ../assets/base.jpg
   :alt: Base direction definition
   :align: center

.. image:: ../assets/topview.jpg
   :alt: Top view reference
   :align: center

.. image:: ../assets/pos_z.jpg
   :alt: Positive z reference
   :align: center

.. image:: ../assets/neg_z.jpg
   :alt: Negative z reference
   :align: center

**Shoulder and elbow (theta2, theta3)**
In side view (xy plane), positive theta2 and theta3 follow the arrow direction:

.. image:: ../assets/sideview.jpg
   :alt: Side view with theta2/theta3 sign convention
   :align: center

Joint Limits
------------
+--------+------------------+
| Joint  | Range            |
+========+==================+
| theta1 | [-90, 90] deg    |
+--------+------------------+
| theta2 | [0, 130] deg     |
+--------+------------------+
| theta3 | [-17, 120] deg   |
+--------+------------------+

Forward Kinematics
------------------
Let R denote the radial projection in the xz plane:

.. math::
   R = L\cos\theta_2 + L\cos\theta_3 + a

Then:

.. math::
   x = R\cos\theta_1 \\
   y = L\sin\theta_2 - L\sin\theta_3 \\
   z = R\sin\theta_1

Inverse Kinematics
------------------
1. **Solve theta1 from x and z**

   .. math::
      R = \sqrt{x^2 + z^2}, \quad \theta_1 = \mathrm{atan2}(z, x)

2. **Reduce to side-plane problem (theta2, theta3)**

   Define:

   .. math::
      K_1 = \frac{R-a}{L}, \quad K_2 = \frac{y}{L}

   Reformulate as:

   .. math::
      A\cos\theta_3 + B\sin\theta_3 = C

   with :math:`A = -2K_1, \quad B = 2K_2, \quad C = -(K_1^2 + K_2^2)`

   Then compute:

   .. math::
      \alpha = \mathrm{atan2}(B, A), \quad \phi = \arccos\left(\frac{C}{\sqrt{A^2 + B^2}}\right)

   .. math::
      \theta_{3,1} = \alpha + \phi, \quad \theta_{3,2} = \alpha - \phi

3. **Recover theta2**

   For each candidate value of theta3:

   .. math::
      \cos\theta_2 = K_1 - \cos\theta_3 \\
      \sin\theta_2 = K_2 + \sin\theta_3 \\
      \theta_2 = \mathrm{atan2}(\sin\theta_2, \cos\theta_2)

Workspace Limits
----------------

**Axis-aligned bounds:**
X: [0, 320], Z: [-320, 320]

**Y bounds from extreme angles:**

.. math::
   Y_{max} = L - L\sin(\theta_{3,min}) \\
   Y_{min} = L\sin(\theta_{2,min}) - L\sin(\theta_{3,max})

.. image:: ../assets/ymin.jpg
   :alt: Minimum y concept
   :align: center

**Radial reach bounds in xz projection:**

.. math::
   R_{min} = L\cos(\theta_{2,max}) + L\cos(\theta_{3,min}) + a \\
   R_{max} = 320

.. image:: ../assets/minreach.jpg
   :alt: Minimum reach concept
   :align: center

.. image:: ../assets/maxreach.jpg
   :alt: Maximum reach concept
   :align: center
