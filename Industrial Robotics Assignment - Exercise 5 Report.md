# Industrial Robotics Assignment - Exercise 5 Report

## 1. Introduction

This report focuses on the kinematic analysis and motion planning of a 6-DOF anthropomorphic manipulator (ENGINEAI-SE01 style). The robot arm consists of a waist, shoulder, elbow, and a 3-DOF spherical wrist. We perform Forward Kinematics (FK), Workspace analysis, Differential Kinematics (Jacobian), Inverse Kinematics (IK), and Trajectory Planning.

## 2. Kinematic Modeling (Exercise 5.1)

The robot is modeled using the Product of Exponentials (PoE) formula.

Parameters:

- Link Lengths: $L_1=0.15$ m, $L_2=0.1$ m, $L_3=0.25$ m, $L_4=0.2$ m.

- Home Configuration ($M$): We define the home configuration as the arm fully extended vertically along the $z_s$-axis.

  

  $$M = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & L_1+L_2+L_3+L_4 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0.7 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Screw Axes ($\mathcal{S}_i$):

Defined in the Space Frame $\{s\}$ located at the base.

1. **Joint 1 (Waist)**: Rotation about $z_s$. $\omega_1=[0,0,1]^T, q_1=[0,0,0]^T$.
2. **Joint 2 (Shoulder)**: Rotation about $y_s$. $\omega_2=[0,1,0]^T, q_2=[0,0,L_1]^T$.
3. **Joint 3 (Elbow)**: Rotation about $y_s$. $\omega_3=[0,1,0]^T, q_3=[0,0,L_1+L_2]^T$.
4. **Joint 4 (Wrist 1)**: Rotation about $z_s$. $\omega_4=[0,0,1]^T, q_4=[0,0,L_1+L_2+L_3]^T$.
5. **Joint 5 (Wrist 2)**: Rotation about $y_s$. $\omega_5=[0,1,0]^T, q_5=[0,0,L_1+L_2+L_3]^T$.
6. **Joint 6 (Wrist 3)**: Rotation about $x_s$. $\omega_6=[1,0,0]^T, q_6=[0,0,L_1+L_2+L_3]^T$.

The Forward Kinematics is given by:



$$T(\theta) = e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2} \dots e^{[\mathcal{S}_6]\theta_6} M$$

Workspace:

Using a Monte Carlo method with joint limits ($\theta_1 \in [-90^\circ, 90^\circ]$, $\theta_2 \in [-70^\circ, 90^\circ]$, $\theta_3 \in [-90^\circ, 90^\circ]$), we visualized the reachable workspace (see Matlab Figure 1).

## 3. Jacobian Analysis (Exercise 5.2)

The Space Jacobian $J_s(\theta)$ maps joint velocities to the end-effector spatial twist: $\mathcal{V}_s = J_s(\theta)\dot{\theta}$.

The columns of the Jacobian are given by:



$$J_{s1} = \mathcal{S}_1$$

$$J_{si} = \text{Ad}(e^{[\mathcal{S}_1]\theta_1} \dots e^{[\mathcal{S}_{i-1}]\theta_{i-1}}) \mathcal{S}_i, \quad i=2\dots6$$



We implemented a custom script to compute this without JacobianSpace. For the configuration $\theta = (50, 10, -15, 30, 30, -15)^\circ$, the resulting end-effector velocity twist was calculated (see code output).

## 4. Inverse Kinematics (Exercise 5.3)

We implemented a **Newton-Raphson** numerical solver.

- **Update Rule**: $\theta_{k+1} = \theta_k + J^\dagger(\theta_k) \mathcal{V}_{err}$

- **Target 1 ($\mathbf{x}_e$)**: Position $(0.2, 0.1, 0.3)$, Orientation defined by Euler Angles $(10^\circ, 30^\circ, 20^\circ)$.

- Target 2 ($T_{se}$): Given matrix.

  The solver successfully converged for both targets.

## 5. Trajectory Planning (Exercise 5.4 & 5.5)

Point-to-Point (P2P):

A cubic time-scaling path was generated between $\mathbf{x}_e$ and $T_{se}$ ($T_f = 0.5$ s). The joint velocity analysis shows that a duration of $0.5$ s results in velocities exceeding the $30^\circ/s$ limit. To satisfy the constraint $|\dot{\theta}| \le 30^\circ/s$, the duration $T_f$ must be significantly increased (approx. factor of 6-10 based on peak velocity).

Multi-Waypoint:

A trajectory passing through Home $\to \mathbf{x}_e \to T_{se} \to$ Home was generated ($T_f=25$ s). The IK was solved at each step, and the joint profiles were plotted, confirming smooth motion within limits.