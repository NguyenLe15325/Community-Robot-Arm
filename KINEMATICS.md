# Kinematic Model and Workspace Constraints

This document presents the robot arm kinematic model, sign conventions, workspace constraints, and forward/inverse kinematic equations used by firmware.

Reference implementation files:
- [firmware/RobotArm/Kinematics3D.h](firmware/RobotArm/Kinematics3D.h)
- [firmware/RobotArm/Kinematics3D.cpp](firmware/RobotArm/Kinematics3D.cpp)
- [firmware/RobotArm/Config_Robot.h](firmware/RobotArm/Config_Robot.h)

Notation and units:
- Position variables x, y, z, L, and a are expressed in mm.
- Home and limit values are listed in deg for readability.
- Trigonometric equations are evaluated in rad internally.
- Joint symbols are denoted as theta1, theta2, and theta3.

## 1. Home Pose and Axis Frame

The home pose is defined by:
- theta1 = 0 deg
- theta2 = 90 deg
- theta3 = 0 deg

Geometric interpretation:
- base (theta1) centered
- lower shank perpendicular to the ground plane
- upper shank parallel to the ground plane
- xyz axes follow the figure below

![Home pose and xyz definition](assets/home.jpg)

## 2. Positive/Negative Rotation Directions

### 2.1 Base axis (theta1)

Base rotation sign follows the base/top-view convention, where pos_z and neg_z indicate the positive/negative theta1 directions.

![Base direction definition](assets/base.jpg)

![Top view reference](assets/topview.jpg)

![Positive z reference](assets/pos_z.jpg)

![Negative z reference](assets/neg_z.jpg)

### 2.2 Shoulder and elbow (theta2, theta3)

In side view (xy plane), positive theta2 and theta3 follow the arrow direction shown in the figure.

![Side view with theta2/theta3 sign convention](assets/sideview.jpg)

## 3. Mechanical Model Used in Firmware

Although the physical Tobler arm uses shoulder/elbow 4-bar linkages, firmware computes motion with an equivalent 3-DOF kinematic model:
- base rotation joint theta1
- shoulder-equivalent joint theta2
- elbow-equivalent joint theta3
- two effective links of length L, plus base offset a

Current model constants:
- L = 140.0 mm
- a = 54.0 mm

## 4. Joint Limits

Joint angle limits used by kinematics:
- theta1 in [-90, 90] deg
- theta2 in [0, 130] deg
- theta3 in [-17, 90] deg

These limits define valid FK/IK solutions and workspace boundaries.

## 5. Forward Kinematics

Let R denote the radial projection in the xz plane:

$$
R = L\cos\theta_2 + L\cos\theta_3 + a
$$

Then Cartesian position is:

$$
x = R\cos\theta_1
$$

$$
y = L\sin\theta_2 - L\sin\theta_3
$$

$$
z = R\sin\theta_1
$$

Forward kinematics is then evaluated directly from the joint state.

## 6. Inverse Kinematics

### 6.1 Solve theta1 from x and z

$$
R = \sqrt{x^2 + z^2}, \quad \theta_1 = \operatorname{atan2}(z, x)
$$

### 6.2 Reduce to side-plane problem (theta2, theta3)

Define:

$$
K_1 = \frac{R-a}{L}, \quad K_2 = \frac{y}{L}
$$

Reformulate as:

$$
A\cos\theta_3 + B\sin\theta_3 = C
$$

with:

$$
A = -2K_1, \quad B = 2K_2, \quad C = -(K_1^2 + K_2^2)
$$

Then compute:

$$
\alpha = \operatorname{atan2}(B, A), \quad \phi = \arccos\left(\frac{C}{\sqrt{A^2 + B^2}}\right)
$$

$$
\theta_{3,1} = \alpha + \phi, \quad \theta_{3,2} = \alpha - \phi
$$

This produces up to two candidate elbow branches.

### 6.3 Recover theta2 and validate

For each candidate value of theta3:

$$
\cos\theta_2 = K_1 - \cos\theta_3
$$

$$
\sin\theta_2 = K_2 + \sin\theta_3
$$

$$
\theta_2 = \operatorname{atan2}(\sin\theta_2, \cos\theta_2)
$$

Both branches are evaluated, and any solution outside joint limits is rejected.

## 7. Workspace Limits

### 7.1 Axis-aligned bounds

Configured axis-aligned bounds:
- X in [0, 320]
- Z in [-320, 320]

Y bounds are derived in the next subsection from joint-angle extremes.

### 7.2 Y bounds from extreme angles

$$
Y_{max} = L - L\sin(\theta_{3,min})
$$

$$
Y_{min} = L\sin(\theta_{2,min}) - L\sin(\theta_{3,max})
$$

The figure below illustrates the minimum-y boundary concept.

![Minimum y concept](assets/ymin.jpg)

### 7.3 Radial reach bounds in xz projection

$$
R_{min} = L\cos(\theta_{2,max}) + L\cos(\theta_{3,min}) + a
$$

$$
R_{max} = 320
$$

Rmax is implemented as an explicit workspace cap in firmware.

The figures below illustrate the minimum and maximum radial reach boundaries.

![Minimum reach concept](assets/minreach.jpg)

![Maximum reach concept](assets/maxreach.jpg)

## 8. Computational Workflow Summary

Kinematic computation workflow:
1. Define positive rotation directions from the geometry figures.
2. Define joint limits from mechanical constraints.
3. Map each joint angle to its motorized axis.
4. Derive forward kinematics from the geometric model.
5. Derive inverse kinematics and apply workspace/joint-limit validity checks from the same model.

This structure keeps geometric documentation and firmware implementation aligned while remaining robust to source refactoring.
