# Inverse-kinematics-2DOF-Robot-arm

To prove the given inverse kinematics equations for a 2 DOF planar robot arm, we need to derive them step by step from the geometry of the system. The equations provided are:

\[
\theta_1 = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right) - \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

\[
\theta_2 = \pm \cos^{-1}\left(\frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}\right)
\]

These equations calculate the joint angles \(\theta_1\) and \(\theta_2\) for a 2 DOF planar robot arm to reach a desired end-effector position \((p_x, p_y)\), where \(l_1\) and \(l_2\) are the lengths of the first and second links, respectively. Let’s derive these equations systematically using the geometry of the robot arm and trigonometric relationships.

---

### Step 1: Understand the 2 DOF Planar Robot Arm
A 2 DOF planar robot arm consists of two links:
- The first link has length \(l_1\) and rotates about the base (origin \((0, 0)\)) at angle \(\theta_1\) relative to the horizontal (x-axis).
- The second link has length \(l_2\) and rotates about the end of the first link at angle \(\theta_2\) relative to the first link.
- The end-effector is at position \((p_x, p_y)\) in the Cartesian plane.

The goal of inverse kinematics is to find \(\theta_1\) and \(\theta_2\) given \((p_x, p_y)\), \(l_1\), and \(l_2\).

---

### Step 2: Forward Kinematics (Position of the End-Effector)
First, let’s write the forward kinematics to relate the joint angles to the end-effector position. The position of the end of the first link (joint 2) is:

\[
x_1 = l_1 \cos \theta_1, \quad y_1 = l_1 \sin \theta_1
\]

The end-effector (end of the second link) is at \((p_x, p_y)\), and its position relative to the base is the sum of the contributions of both links:

\[
p_x = l_1 \cos \theta_1 + l_2 \cos (\theta_1 + \theta_2)
\]

\[
p_y = l_1 \sin \theta_1 + l_2 \sin (\theta_1 + \theta_2)
\]

Our task in inverse kinematics is to solve these equations for \(\theta_1\) and \(\theta_2\) given \((p_x, p_y)\).

---

### Step 3: Derive \(\theta_2\) Using the Law of Cosines
To find \(\theta_2\), we eliminate \(\theta_1\) by working with the distance from the base to the end-effector. Square both forward kinematics equations and add them:

\[
p_x^2 + p_y^2 = (l_1 \cos \theta_1 + l_2 \cos (\theta_1 + \theta_2))^2 + (l_1 \sin \theta_1 + l_2 \sin (\theta_1 + \theta_2))^2
\]

Expand the right-hand side:

\[
p_x^2 + p_y^2 = (l_1^2 \cos^2 \theta_1 + 2 l_1 l_2 \cos \theta_1 \cos (\theta_1 + \theta_2) + l_2^2 \cos^2 (\theta_1 + \theta_2)) + (l_1^2 \sin^2 \theta_1 + 2 l_1 l_2 \sin \theta_1 \sin (\theta_1 + \theta_2) + l_2^2 \sin^2 (\theta_1 + \theta_2))
\]

Simplify using \(\cos^2 \theta + \sin^2 \theta = 1\):

\[
p_x^2 + p_y^2 = l_1^2 (\cos^2 \theta_1 + \sin^2 \theta_1) + l_2^2 (\cos^2 (\theta_1 + \theta_2) + \sin^2 (\theta_1 + \theta_2)) + 2 l_1 l_2 (\cos \theta_1 \cos (\theta_1 + \theta_2) + \sin \theta_1 \sin (\theta_1 + \theta_2))
\]

\[
p_x^2 + p_y^2 = l_1^2 + l_2^2 + 2 l_1 l_2 (\cos \theta_1 \cos (\theta_1 + \theta_2) + \sin \theta_1 \sin (\theta_1 + \theta_2))
\]

Use the cosine angle addition formula:

\[
\cos (\theta_1 + \theta_2) = \cos \theta_1 \cos \theta_2 - \sin \theta_1 \sin \theta_2
\]

\[
\sin (\theta_1 + \theta_2) = \sin \theta_1 \cos \theta_2 + \cos \theta_1 \sin \theta_2
\]

So:

\[
\cos \theta_1 \cos (\theta_1 + \theta_2) + \sin \theta_1 \sin (\theta_1 + \theta_2) = \cos \theta_1 (\cos \theta_1 \cos \theta_2 - \sin \theta_1 \sin \theta_2) + \sin \theta_1 (\sin \theta_1 \cos \theta_2 + \cos \theta_1 \sin \theta_2)
\]

\[
= \cos^2 \theta_1 \cos \theta_2 - \cos \theta_1 \sin \theta_1 \sin \theta_2 + \sin^2 \theta_1 \cos \theta_2 + \sin \theta_1 \cos \theta_1 \sin \theta_2
\]

\[
= (\cos^2 \theta_1 + \sin^2 \theta_1) \cos \theta_2 + (\sin \theta_1 \cos \theta_1 - \cos \theta_1 \sin \theta_1) \sin \theta_2
\]

\[
= \cos \theta_2
\]

Thus:

\[
p_x^2 + p_y^2 = l_1^2 + l_2^2 + 2 l_1 l_2 \cos \theta_2
\]

Rearrange to solve for \(\cos \theta_2\):

\[
p_x^2 + p_y^2 - l_1^2 - l_2^2 = 2 l_1 l_2 \cos \theta_2
\]

\[
\cos \theta_2 = \frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}
\]

\[
\theta_2 = \cos^{-1}\left(\frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}\right)
\]

Since \(\cos \theta = \cos (-\theta)\), we get two solutions:

\[
\theta_2 = \pm \cos^{-1}\left(\frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}\right)
\]

This matches the given equation for \(\theta_2\), corresponding to the "elbow-up" (\(+\)) and "elbow-down" (\(-\)) configurations. This completes the derivation of the second equation.

---

### Step 4: Derive \(\theta_1\)
Now that we have \(\theta_2\), we need to find \(\theta_1\). Return to the forward kinematics equations:

\[
p_x = l_1 \cos \theta_1 + l_2 \cos (\theta_1 + \theta_2)
\]

\[
p_y = l_1 \sin \theta_1 + l_2 \sin (\theta_1 + \theta_2)
\]

Let’s introduce an auxiliary angle \(\phi\), where \(\phi\) is the angle that the line from the base to the end-effector makes with the x-axis:

\[
\tan \phi = \frac{p_y}{p_x}, \quad \sqrt{p_x^2 + p_y^2} = r
\]

\[
p_x = r \cos \phi, \quad p_y = r \sin \phi
\]

\[
\sin \phi = \frac{p_y}{\sqrt{p_x^2 + p_y^2}}, \quad \cos \phi = \frac{p_x}{\sqrt{p_x^2 + p_y^2}}
\]

Substitute into the forward kinematics:

\[
r \cos \phi = l_1 \cos \theta_1 + l_2 \cos (\theta_1 + \theta_2)
\]

\[
r \sin \phi = l_1 \sin \theta_1 + l_2 \sin (\theta_1 + \theta_2)
\]

Expand \(\cos (\theta_1 + \theta_2)\) and \(\sin (\theta_1 + \theta_2)\):

\[
r \cos \phi = l_1 \cos \theta_1 + l_2 (\cos \theta_1 \cos \theta_2 - \sin \theta_1 \sin \theta_2)
\]

\[
r \sin \phi = l_1 \sin \theta_1 + l_2 (\sin \theta_1 \cos \theta_2 + \cos \theta_1 \sin \theta_2)
\]

Group terms:

\[
r \cos \phi = (l_1 + l_2 \cos \theta_2) \cos \theta_1 - (l_2 \sin \theta_2) \sin \theta_1
\]

\[
r \sin \phi = (l_1 + l_2 \cos \theta_2) \sin \theta_1 + (l_2 \sin \theta_2) \cos \theta_1
\]

This is a system of equations of the form:

\[
a \cos \theta_1 - b \sin \theta_1 = c
\]

\[
a \sin \theta_1 + b \cos \theta_1 = d
\]

Where:

\[
a = l_1 + l_2 \cos \theta_2, \quad b = l_2 \sin \theta_2, \quad c = r \cos \phi, \quad d = r \sin \phi
\]

To solve for \(\theta_1\), let’s express \(\theta_1\) using the tangent of an angle. Multiply the first equation by \(b\) and the second by \(a\), then add:

\[
b (a \cos \theta_1 - b \sin \theta_1) = b c
\]

\[
a (a \sin \theta_1 + b \cos \theta_1) = a d
\]

\[
a b \cos \theta_1 - b^2 \sin \theta_1 + a^2 \sin \theta_1 + a b \cos \theta_1 = b c + a d
\]

\[
(a^2 + b^2) \sin \theta_1 + 2 a b \cos \theta_1 - (b c + a d) = 0
\]

Now multiply the first by \(a\) and the second by \(b\), then subtract:

\[
a (a \cos \theta_1 - b \sin \theta_1) = a c
\]

\[
b (a \sin \theta_1 + b \cos \theta_1) = b d
\]

\[
a^2 \cos \theta_1 - a b \sin \theta_1 - (a b \sin \theta_1 + b^2 \cos \theta_1) = a c - b d
\]

\[
(a^2 + b^2) \cos \theta_1 - 2 a b \sin \theta_1 - (a c - b d) = 0
\]

Now we have:

\[
(a^2 + b^2) \sin \theta_1 + 2 a b \cos \theta_1 = b c + a d
\]

\[
(a^2 + b^2) \cos \theta_1 - 2 a b \sin \theta_1 = a c - b d
\]

Divide these equations:

\[
\frac{\sin \theta_1}{\cos \theta_1} = \frac{b c + a d}{a c - b d}
\]

\[
\tan \theta_1 = \frac{b c + a d}{a c - b d}
\]

Substitute \(a\), \(b\), \(c\), and \(d\):

\[
a = l_1 + l_2 \cos \theta_2, \quad b = l_2 \sin \theta_2, \quad c = r \cos \phi, \quad d = r \sin \phi
\]

\[
b c + a d = (l_2 \sin \theta_2)(r \cos \phi) + (l_1 + l_2 \cos \theta_2)(r \sin \phi)
\]

\[
a c - b d = (l_1 + l_2 \cos \theta_2)(r \cos \phi) - (l_2 \sin \theta_2)(r \sin \phi)
\]

This form is complex, so let’s try a geometric approach to derive \(\theta_1\) more directly, matching the given equation.

---

### Step 5: Geometric Approach for \(\theta_1\)
Consider the geometry: the end-effector is at \((p_x, p_y)\), and the end of the first link is at \((x_1, y_1) = (l_1 \cos \theta_1, l_1 \sin \theta_1)\). The vector from the end of the first link to the end-effector is along the second link, with length \(l_2\), at angle \(\theta_1 + \theta_2\).

The angle \(\phi\) (from the base to the end-effector) satisfies:

\[
\sin \phi = \frac{p_y}{\sqrt{p_x^2 + p_y^2}}
\]

\[
\phi = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right)
\]

Now, consider the triangle formed by the base, the end of the first link, and the end-effector. The angle between the line from the base to the end-effector (at angle \(\phi\)) and the first link (at angle \(\theta_1\)) is \(\phi - \theta_1\). Apply the law of sines in this triangle:

- Side from base to end-effector: \(\sqrt{p_x^2 + p_y^2}\)
- Side from base to end of first link: \(l_1\)
- Side from end of first link to end-effector: \(l_2\)

The angle opposite \(l_2\) is \(\phi - \theta_1\). The angle opposite \(\sqrt{p_x^2 + p_y^2}\) is the angle at the end-effector, which we can relate to \(\theta_2\). Using the law of sines:

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
\theta_1 = \phi - \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

\[
\theta_1 = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right) - \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

This matches the given equation for \(\theta_1\).

---

### Step 6: Verify the Equations
- The \(\theta_2\) equation was derived using the law of cosines, matching the given form.
- The \(\theta_1\) equation was derived geometrically, confirming the given expression.

Both equations are standard for the inverse kinematics of a 2 DOF planar robot arm. The \(\pm\) in \(\theta_2\) accounts for the two possible configurations (elbow-up and elbow-down), and \(\theta_1\) adjusts accordingly.

---

### Conclusion
We have derived the given inverse kinematics equations for a 2 DOF planar robot arm:

\[
\theta_2 = \pm \cos^{-1}\left(\frac{p_x^2 + p_y^2 - l_1^2 - l_2^2}{2 l_1 l_2}\right)
\]

\[
\theta_1 = \sin^{-1}\left(\frac{p_y}{\sqrt{p_x^2 + p_y^2}}\right) - \sin^{-1}\left(\frac{l_2 \sin \theta_2}{\sqrt{p_x^2 + p_y^2}}\right)
\]

These equations are correct and consistent with the geometry of the system. They allow the calculation of joint angles to position the end-effector at \((p_x, p_y)\), assuming the position is reachable (\(\sqrt{p_x^2 + p_y^2} \leq l_1 + l_2\)).
