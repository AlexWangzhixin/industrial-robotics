# Industrial Robotics Assignment - Exercise 4 Report

## 1. Introduction

This report addresses the kinematic analysis and trajectory generation for a hyper-redundant 7-DOF manipulator. The tasks involve solving inverse kinematics (IK) for linear and point-to-point motions, as well as implementing and deriving different time-scaling laws (Cubic, Triangular, and Trapezoidal) for trajectory planning.

## 2. Kinematic Modeling (7-DOF Manipulator)

Based on the schematic in Figure 4, we model the robot as a 7-DOF serial manipulator with revolute joints. We assume a standard "S-R-S" (Shoulder-Elbow-Wrist) configuration where joint axes alternate.

Assumptions:

- Link Length parameter $L = 0.1$ m.

- Home Configuration ($M$): The robot is fully extended vertically along the $z$-axis.

  

  $$M = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 7L \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

- **Screw Axes ($\mathcal{S}$)** in the Space Frame:

  1. Joint 1 (Base Rotation): $\omega=[0,0,1]$, $q=[0,0,0]$.
  2. Joint 2 (Shoulder Pitch): $\omega=[0,1,0]$, $q=[0,0,L]$.
  3. Joint 3 (Shoulder Roll): $\omega=[0,0,1]$, $q=[0,0,2L]$.
  4. Joint 4 (Elbow Pitch): $\omega=[0,1,0]$, $q=[0,0,3L]$.
  5. Joint 5 (Wrist Roll): $\omega=[0,0,1]$, $q=[0,0,4L]$.
  6. Joint 6 (Wrist Pitch): $\omega=[0,1,0]$, $q=[0,0,5L]$.
  7. Joint 7 (Wrist Roll): $\omega=[0,0,1]$, $q=[0,0,6L]$.

## 3. Exercise 4.1: Linear Motion & Numerical IK

Problem: Follow a linear path with velocity $\dot{x} = [-0.1, 0.0, 0.05]^T$ m/s for $T=2$s, starting from a given pose $T_{start}$.

Method:

We implemented a Newton-Raphson iterative solver to find the joint angles $\theta$. Since the robot is redundant (7 DoF), the Jacobian $J(\theta)$ is non-square ($6 \times 7$). We use the Moore-Penrose Pseudoinverse ($J^\dagger = J^T(JJ^T)^{-1}$) to resolve the redundancy, which minimizes the norm of joint velocities $||\dot{\theta}||$.

The update rule is:



$$\theta_{k+1} = \theta_k + J^\dagger(\theta_k) \mathcal{V}_b$$



where $\mathcal{V}_b$ is the body twist representing the error between current and desired pose.

## 4. Exercise 4.3: Triangular Time-Scaling

Derivation:

A triangular velocity profile consists of a linear acceleration phase up to $t = T_f/2$ and a symmetric deceleration phase.

- Constraint: Total displacement $s(T_f) = 1$.

- $v_{max}$ occurs at $T_f/2$. The area under the $v-t$ graph is displacement.

  

  $$\text{Area} = \frac{1}{2} \cdot T_f \cdot v_{max} = 1 \implies v_{max} = \frac{2}{T_f}$$

- **Equations for $s(t)$:**

  - For $0 \le t \le T_f/2$: Constant acceleration $a = \frac{v_{max}}{T_f/2} = \frac{4}{T_f^2}$.

    

    $$s(t) = \frac{1}{2} a t^2 = \frac{2}{T_f^2} t^2$$

  - For $T_f/2 < t \le T_f$: Deceleration.

    

    $$s(t) = 1 - \frac{2}{T_f^2} (T_f - t)^2$$

## 5. Exercise 4.4: Trapezoidal Time-Scaling

Derivation:

We impose a trapezoidal velocity profile with total duration $T_f$ and rise time $t^*$ (acceleration duration).

- **Phases:**

  1. $0 \le t \le t^*$: Acceleration.
  2. $t^* < t < T_f - t^*$: Constant velocity ($v_{max}$).
  3. $T_f - t^* \le t \le T_f$: Deceleration.

- Determining $v_{max}$:

  The area under the trapezoid must equal 1.

  

  $$\text{Area} = \frac{1}{2}(T_f + (T_f - 2t^*)) \cdot v_{max} = (T_f - t^*) v_{max} = 1$$

  $$\therefore v_{max} = \frac{1}{T_f - t^*}$$

- **Equations for $s(t)$:**

  - Acceleration $a = \frac{v_{max}}{t^*} = \frac{1}{t^*(T_f - t^*)}$.

  1. **Phase 1:** $s(t) = \frac{1}{2} a t^2$.

  2. Phase 2: $s(t) = s(t^*) + v_{max}(t - t^*)$.

     (where $s(t^*) = \frac{1}{2} a (t^*)^2 = \frac{v_{max} t^*}{2}$)

  3. **Phase 3:** $s(t) = 1 - \frac{1}{2} a (T_f - t)^2$.