# 2 DOF Planar Robot Arm Inverse Kinematics

This project provides the inverse kinematics equations for a 2 Degree-of-Freedom (DOF) planar robot arm. These equations calculate the joint angles \(\theta_1\) and \(\theta_2\) required to position the end-effector at a specific point \((p_x, p_y)\) in a 2D plane.

## Overview

A 2 DOF planar robot arm consists of two links:
- **Link 1**: Length \(l_1\), rotates about the base at angle \(\theta_1\) (relative to the x-axis).
- **Link 2**: Length \(l_2\), rotates about the end of the first link at angle \(\theta_2\) (relative to the first link).

The goal of inverse kinematics is to determine \(\theta_1\) and \(\theta_2\) given the desired end-effector position \((p_x, p_y)\).

## Inverse Kinematics Equations

The inverse kinematics equations for the 2 DOF planar robot arm are:

\[
\theta_1 = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right) - \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

\[
\theta_2 = \pm \cos^{-1}\left(\frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}\right)
\]

### Parameters:
- \(p_x, p_y\): Cartesian coordinates of the end-effector.
- \(l_1\): Length of the first link.
- \(l_2\): Length of the second link.
- \(\theta_1\): Angle of the first link relative to the x-axis.
- \(\theta_2\): Angle of the second link relative to the first link.

### Notes:
- The \(\pm\) in the \(\theta_2\) equation indicates two possible configurations: "elbow-up" (\(+\)) and "elbow-down" (\(-\)).
- The position \((p_x, p_y)\) must be reachable, i.e., \(\sqrt{p_x^2 + p_y^2} \leq l_1 + l_2\).

## Derivation

### \(\theta_2\) Derivation
The equation for \(\theta_2\) is derived using the law of cosines. Consider the triangle formed by:
- The base at \((0, 0)\),
- The end of the first link,
- The end-effector at \((p_x, p_y)\).

The distance from the base to the end-effector is \(\sqrt{p_x^2 + p_y^2}\). Using the law of cosines:

\[
p_x^2 + p_y^2 = l_1^2 + l_2^2 + 2 l_1 l_2 \cos \theta_2
\]

Rearrange to solve for \(\theta_2\):

\[
\cos \theta_2 = \frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}
\]

\[
\theta_2 = \pm \cos^{-1}\left(\frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}\right)
\]

### \(\theta_1\) Derivation
The equation for \(\theta_1\) is derived geometrically. Define \(\phi\) as the angle from the base to the end-effector:

\[
\phi = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right)
\]

Using the law of sines in the triangle formed by the base, the end of the first link, and the end-effector:

\[
\frac{\sin (\phi - \theta_1)}{l_2} = \frac{\sin \theta_2}{\sqrt{p_x^2 + p_y^2}}
\]

\[
\sin (\phi - \theta_1) = \frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}
\]

\[
\phi - \theta_1 = \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

\[
\theta_1 = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right) - \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

## Usage
These equations can be used to:
1. Compute joint angles for a single end-effector position.
2. Generate a trajectory by calculating \(\theta_1\) and \(\theta_2\) for a sequence of \((p_x, p_y)\) points along a desired path (e.g., a straight line or circle).

### Example
To reach a position \((p_x, p_y) = (2, 3)\) with \(l_1 = 2\) and \(l_2 = 1\):
1. Compute \(\theta_2\) using the second equation.
2. Substitute \(\theta_2\) into the first equation to find \(\theta_1\).

## Limitations
- The equations assume the position is reachable (\(\sqrt{p_x^2 + p_y^2} \leq l_1 + l_2\)).
- Singularities may occur when the arm is fully extended or folded.
- Choose the appropriate sign for \(\theta_2\) based on the desired configuration.

## License
This project is licensed under the MIT License.
