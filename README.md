# 2-DoF Robot Arm Project

Cre: https://www.mathworks.com/matlabcentral/fileexchange/75175-simulation-of-a-2-dof-manipulator-using-simscape-multibody/

## Overview
This project focuses on the modeling, analysis, and control of a 2-Degrees-of-Freedom (2-DoF) robot arm. The primary objectives include deriving the inverse kinematics for the robot arm, planning its trajectory using a third-order polynomial, and simulating the system using MATLAB/Simulink. The project also involves generating a trajectory to draw the sequence of characters "21ECE" using the robot arm.

## Table of Contents
- [Project Description](#project-description)
- [2-DoF Robot Model](#2-dof-robot-model)
- [Inverse Kinematics](#inverse-kinematics)
- [Trajectory Planning](#trajectory-planning)
- [MATLAB/Simulink Implementation](#matlab-simulink-implementation)
- [Drawing the Letters "21ECE"](#drawing-the-letters-21ece)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Project Description
The 2-DoF robot arm is a planar manipulator with two joints, characterized by joint angles \( q_1 \) and \( q_2 \). The project addresses the following key aspects:
- **Inverse Kinematics**: Calculating the joint angles \( q_1 \) and \( q_2 \) given the desired end-effector position \((x, y)\).
- **Trajectory Planning**: Using a third-order polynomial to generate smooth trajectories for the robot arm.
- **Simulation**: Implementing the robot arm model and control in MATLAB/Simulink to simulate its motion.
- **Application**: Programming the robot arm to draw the sequence of characters "21ECE" in a 2D plane.

## 2-DoF Robot Model
The robot arm is modeled as a planar manipulator with two links of lengths \( l_1 = 4 \) and \( l_2 = 4 \). The pose of the end-effector is described by Cartesian coordinates \((x, y)\) in the world coordinate frame. The configuration used is the "elbow-up" posture.

## Inverse Kinematics
The inverse kinematics problem involves determining the joint angles \( q_1 \) and \( q_2 \) for a given end-effector position \((x, y)\). The equations are:

\[
q_2 = \cos^{-1}\left(\frac{x^2 + y^2 - a_1^2 - a_2^2}{2 a_1 a_2}\right)
\]

\[
q_1 = \tan^{-1}\left(\frac{y}{x}\right) - \tan^{-1}\left(\frac{a_2 \sin q_2}{a_1 + a_2 \cos q_2}\right)
\]

where \( a_1 = l_1 = 4 \), \( a_2 = l_2 = 4 \). The MATLAB function `InverseKinematics.m` implements these equations.

## Trajectory Planning
Trajectory planning is achieved using a third-order polynomial to ensure smooth motion:

\[
q(t) = a_3 t^3 + a_2 t^2 + a_1 t + a_0
\]

The coefficients \( a_0, a_1, a_2, a_3 \) are determined based on initial and final conditions for position and velocity:

\[
q(t_0) = q_i, \quad q(t_f) = q_f, \quad \dot{q}(t_0) = \dot{q}_i, \quad \dot{q}(t_f) = \dot{q}_f
\]

For the case where \( t_0 = 0 \), the coefficients are:

\[
a_0 = q_i
\]

\[
a_1 = \dot{q}_i
\]

\[
a_2 = \frac{3(q_f - q_i) - (2\dot{q}_i + \dot{q}_f)t_f}{t_f^2}
\]

\[
a_3 = \frac{-2(q_f - q_i) + (\dot{q}_i + \dot{q}_f)t_f}{t_f^3}
\]

An example trajectory is provided with initial conditions \( q(0) = 10 \), \( \dot{q}(0) = 0 \), final conditions \( q(t_f) = 45 \), \( \dot{q}(t_f) = 0 \), resulting in:

\[
q(t) = 10 + 6.5625 t^2 - 1.0938 t^3
\]

The MATLAB script `TinhQuyDao.m` implements this trajectory planning.

## MATLAB/Simulink Implementation
The project includes MATLAB scripts and Simulink models for simulation:
- **InverseKinematics.m**: Computes joint angles for a given end-effector position.
- **TinhQuyDao.m**: Calculates the coefficients for the third-order polynomial trajectory.
- **Simulink Model**: Visualizes the robot arm's motion using the Mechanics Explorer in Simulink.

## Drawing the Letters "21ECE"
The robot arm is programmed to draw the sequence "21ECE" by generating a series of \((x, y)\) coordinates corresponding to the shapes of these characters. The inverse kinematics function is used to compute the required joint angles, and the third-order polynomial ensures smooth transitions between points.

## Repository Structure
```
2-DoF-Robot-Arm/
├── InverseKinematics.m        # MATLAB function for inverse kinematics
├── TinhQuyDao.m              # MATLAB script for trajectory planning
├── Simulink_Model.slx        # Simulink model for robot arm simulation
├── Robot_Arm_2DoF.pdf        # Documentation with model details and equations
├── README.md                 # Project overview and instructions
```

## Requirements
- MATLAB (with Simulink and Robotics Toolbox)
- Basic understanding of robotics, inverse kinematics, and trajectory planning

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/2-DoF-Robot-Arm.git
   ```
2. Ensure MATLAB is installed with the required toolboxes.
3. Open MATLAB and navigate to the project directory.

## Usage
1. Run `InverseKinematics.m` to compute joint angles for a desired \((x, y)\) position.
2. Use `TinhQuyDao.m` to generate a smooth trajectory for the robot arm.
3. Open `Simulink_Model.slx` in Simulink to simulate and visualize the robot arm's motion.
4. Modify the scripts to input coordinates for drawing "21ECE" or other custom paths.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue to discuss improvements or bug fixes.

## License
This project is licensed under the MIT License.
