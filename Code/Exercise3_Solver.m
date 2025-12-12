%% Industrial Robotics Assignment - Exercise 3
% Description: Unitree Robot Leg Kinematics with Library Integration
% Requires: Industrial_Robotics_Library (JacobianSpace, etc.)

clear all; close all; clc;

% Add path to library if necessary
addpath('../Industrial_Robotics_Library/');

%% 1. Parameters & Setup
L1 = 0.3; % [m] Upper leg
L2 = 0.4; % [m] Lower leg
L0 = 0.4; % [m] CoM to Joint 1 offset

% Offsets
p_J1_S = [L0; 0; 0];      % Joint 1 position in Space Frame {S}
p_J1_F = [0; 0.5; 0];     % Joint 1 position in Foot Frame {F}
p_F_J1 = -p_J1_F;         % Foot origin in Joint 1 Frame

% Ellipse parameters (from problem description)
x1 = 0; y1 = 0;          % Start point in {F}
x2 = 0.25; y2 = 0.2;     % End point in {F}
e = 0.9;                 % Eccentricity

% Calculated ellipse parameters
dist = sqrt((x2-x1)^2 + (y2-y1)^2);
a = dist / 2;
b = a * sqrt(1 - e^2);
w = atan2(y2-y1, x2-x1);

fprintf('Ellipse parameters: a = %.4f, b = %.4f, w = %.4f rad\n', a, b, w);

% Parametric Trajectory Function (beta from pi to 0)
% Returns position relative to {F} origin
get_traj = @(beta) [ ...
    (x1+x2)/2 + a*cos(beta)*cos(w) - b*sin(beta)*sin(w); ...
    (y1+y2)/2 + a*cos(beta)*sin(w) + b*sin(beta)*cos(w); ...
    0 ]; % z=0 for planar

% Plot trajectory shape
beta_plot = linspace(0, 2*pi, 100);
traj_points = zeros(3, length(beta_plot));
for i = 1:length(beta_plot)
    traj_points(:,i) = get_traj(beta_plot(i));
end

figure(1); 
plot(traj_points(1,:), traj_points(2,:), 'b--'); hold on;
plot(x1, y1, 'ko', 'MarkerFaceColor', 'k'); text(x1, y1-0.05, 'Start');
plot(x2, y2, 'ro', 'MarkerFaceColor', 'r'); text(x2, y2+0.05, 'End');
axis equal; grid on;
title('Exercise 3.1: Foot Trajectory (in Foot Frame)');
xlabel('x [m]'); ylabel('y [m]');

%% 2. Analytical Inverse Kinematics (Exercise 3.2)
fprintf('\n--- 3.2 Analytical IK ---\n');

% Define targets in {J1} frame
P1_F = [x1; y1; 0];      % Start position in {F}
P2_F = [x2; y2; 0];      % End position in {F}
P1_J1 = P1_F + p_F_J1;   % in {J1}
P2_J1 = P2_F + p_F_J1;   % in {J1}

% Solve analytical IK for start and end positions
theta_start = analytical_IK(P1_J1(1), P1_J1(2), L1, L2);
theta_end = analytical_IK(P2_J1(1), P2_J1(2), L1, L2);

fprintf('Configuration 1 (Start): x=%.2f, y=%.2f -> theta1=%.2f deg, theta2=%.2f deg\n', ...
    P1_J1(1), P1_J1(2), rad2deg(theta_start(1)), rad2deg(theta_start(2)));
fprintf('Configuration 2 (End):   x=%.2f, y=%.2f -> theta1=%.2f deg, theta2=%.2f deg\n', ...
    P2_J1(1), P2_J1(2), rad2deg(theta_end(1)), rad2deg(theta_end(2)));

%% 3. Velocity Search and Simulation (Exercise 3.3, 3.4, 3.5)
fprintf('\n--- 3.3 & 3.4 Numerical IK and Velocity Check ---\n');

% Constraints
theta_dot_max_limit = 5.0; % [rad/s]
beta_dot_guess = 1.0;      % Initial guess for beta_dot [rad/s]

% Variables for storing results
history.time = [];
history.pos_S = [];     % Foot position in Space Frame
history.theta = [];     % Joint angles
history.vel_foot = [];  % Foot Cartesian Velocity
history.vel_joint = []; % Joint Velocities

% Velocity Search Loop to find maximum beta_dot that satisfies constraints
found_valid_beta_dot = false;
beta_dot = beta_dot_guess;

while ~found_valid_beta_dot
    % Simulation setup
    dt = 0.02; 
    T_total = pi / beta_dot; % Time to travel pi radians
    time = 0:dt:T_total;
    
    max_q_vel = 0;
    valid_run = true;
    
    for i = 1:length(time)
        t = time(i);
        beta = pi - beta_dot * t; % Current beta (linear time scaling)
        
        % Desired Position and Velocity
        p_d_F = get_traj(beta);       % in {F}
        p_d_J1 = p_d_F + p_F_J1;      % in {J1}
        
        % Cartesian Velocity from Ellipse derivative (Chain rule)
        dx_dbeta = -a*sin(beta)*cos(w) - b*cos(beta)*sin(w);
        dy_dbeta = -a*sin(beta)*sin(w) + b*cos(beta)*cos(w);
        v_d_J1 = [dx_dbeta; dy_dbeta] * (-beta_dot);
        
        % Calculate joint velocities using Jacobian
        % Use analytical solution for current position
        theta_curr = analytical_IK(p_d_J1(1), p_d_J1(2), L1, L2);
        
        % Calculate Jacobian matrix
        J = get_jacobian_2R(theta_curr, L1, L2);
        
        % Solve for joint velocities
        q_dot = J \ v_d_J1;
        
        % Check singularity
        if cond(J) > 1e4
            valid_run = false;
            break;
        end
        
        % Check max velocity constraint
        current_max_vel = max(abs(q_dot));
        if current_max_vel > max_q_vel
            max_q_vel = current_max_vel;
        end
        
        if current_max_vel > theta_dot_max_limit
            valid_run = false;
            break;
        end
    end
    
    fprintf('Testing beta_dot = %.2f rad/s -> Max Joint Vel = %.2f rad/s\n', ...
        beta_dot, max_q_vel);
    
    if valid_run
        found_valid_beta_dot = true;
        fprintf('>>> SUCCESS! beta_dot = %.2f satisfies constraints.\n', beta_dot);
    else
        % Reduce beta_dot and try again
        beta_dot = beta_dot * 0.8;
    end
end

%% 4. Generate Full Trajectory and Calculate Velocities
dt = 0.02; 
T_total = pi / beta_dot;
time = 0:dt:T_total;

% Define Screw Axes for Home Config (Horizontal Extension)
% J1 at (L0,0), J2 at (L0+L1,0). Rot axes = Z (0,0,1).
S1 = [0; 0; 1; cross(-[0;0;1], [L0; 0; 0])];       % [0 0 1 0 -L0 0]
S2 = [0; 0; 1; cross(-[0;0;1], [L0+L1; 0; 0])];    % [0 0 1 0 -(L0+L1) 0]
Slist = [S1, S2];

for i = 1:length(time)
    t = time(i);
    beta = pi - beta_dot * t;
    
    % Desired Position
    p_d_F = get_traj(beta);       % in {F}
    p_d_J1 = p_d_F + p_F_J1;      % in {J1}
    p_d_S = p_d_J1 + p_J1_S;      % in {S} (Space Frame)
    
    % Inverse Kinematics (Analytical for position)
    theta_sol = analytical_IK(p_d_J1(1), p_d_J1(2), L1, L2);
    
    % Velocity Calculation
    % Cartesian Velocity from Ellipse derivative
    dx_dbeta = -a*sin(beta)*cos(w) - b*cos(beta)*sin(w);
    dy_dbeta = -a*sin(beta)*sin(w) + b*cos(beta)*cos(w);
    v_cart_F = [dx_dbeta; dy_dbeta; 0] * (-beta_dot);
    
    % Joint Velocity using Library JacobianSpace
    Js = JacobianSpace(Slist, theta_sol);
    
    % Extract linear part of Jacobian and solve for joint velocities
    J_linear = Js(4:6, :);
    if det(J_linear'*J_linear) < 1e-6
        warning('Near Singularity at t=%.2f', t);
        theta_dot = [0;0];
    else
        theta_dot = pinv(J_linear) * v_cart_F;
    end
    
    % Store Data
    history.time = [history.time, t];
    history.pos_S = [history.pos_S, p_d_S];
    history.theta = [history.theta, theta_sol];
    history.vel_foot = [history.vel_foot, v_cart_F];
    history.vel_joint = [history.vel_joint, theta_dot];
end

fprintf('\nSimulation Complete.\n');

%% 5. Results Visualization

% Plot Foot Position and Joint Angles vs Time
figure(2);
subplot(2,1,1);
plot(history.time, history.pos_S(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(history.time, history.pos_S(2,:), 'b', 'LineWidth', 1.5);
title('Foot Position w.r.t CoM Frame');
legend('x_s', 'y_s'); grid on; xlabel('Time [s]'); ylabel('Position [m]');

subplot(2,1,2);
plot(history.time, rad2deg(history.theta(1,:)), 'r', 'LineWidth', 1.5); hold on;
plot(history.time, rad2deg(history.theta(2,:)), 'b', 'LineWidth', 1.5);
title('Joint Angles vs Time');
legend('\theta_1', '\theta_2'); grid on; xlabel('Time [s]'); ylabel('Angle [deg]');

% 3D Visualization (Exercise 3.5)
figure('Name', 'Exercise 3.5: 3D Leg Trajectory', 'Color', 'w');
hold on; grid on; axis equal;
view(3);
xlabel('X_S [m]'); ylabel('Y_S [m]'); zlabel('Z_S [m]');
title('3D Trajectory and Velocity Vectors');

% Draw Robot Body (Schematic)
plot3([-0.2, 0.6], [0, 0], [0, 0], 'k-', 'LineWidth', 5);
text(0, 0, 0.1, 'Body (CoM)');

% Draw Trajectory
plot3(history.pos_S(1,:), history.pos_S(2,:), history.pos_S(3,:), 'b--', 'LineWidth', 1.5);

% Plot Velocity Vectors at selected points
n_steps = length(history.time);
skip = 5; % Plot every 5th frame for vectors

for i = 1:skip:n_steps
    q = history.theta(:, i);
    pos = history.pos_S(:, i);
    vel = history.vel_foot(:, i);
    
    % Forward Kinematics to get joint positions for plotting
    p0 = p_J1_S;
    p1 = p0 + [L1*cos(q(1)); L1*sin(q(1)); 0];
    p2 = pos; % End effector
    
    % Plot Links
    h_links = plot3([p0(1) p1(1) p2(1)], [p0(2) p1(2) p2(2)], [0 0 0], 'o-', ...
        'Color', [0.8 0.2 0.2], 'LineWidth', 2, 'MarkerFaceColor', 'k');
    
    % Plot Velocity Vector
    h_vel = quiver3(p2(1), p2(2), p2(3), vel(1), vel(2), vel(3), ...
        0.5, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    drawnow;
    
    % Clear specific handles for animation effect
    if i < n_steps - skip
        delete(h_links);
        delete(h_vel);
    end
end

%% Helper Functions

function theta = analytical_IK(x, y, L1, L2)
    % Solves 2R IK for x,y in the base frame of the chain
    % Returns [theta1; theta2]
    
    % Check workspace
    r = sqrt(x^2 + y^2);
    if r > (L1 + L2) || r < abs(L1 - L2)
        warning('Target out of workspace');
        theta = [NaN; NaN];
        return;
    end
    
    % Law of cosines for theta2
    cos_t2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    
    % Safety for acos
    cos_t2 = max(min(cos_t2, 1.0), -1.0);
    
    % Choose knee-backward solution (theta2 < 0 for dog leg)
    theta2 = -acos(cos_t2);
    
    % Calculate theta1
    k1 = L1 + L2*cos(theta2);
    k2 = L2*sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    theta = [theta1; theta2];
end

function J = get_jacobian_2R(q, L1, L2)
    % Calculate 2R planar robot Jacobian matrix
    t1 = q(1);
    t2 = q(2);
    s1 = sin(t1); c1 = cos(t1);
    s12 = sin(t1+t2); c12 = cos(t1+t2);
    
    J = [ -L1*s1 - L2*s12,  -L2*s12;
           L1*c1 + L2*c12,   L2*c12 ];
end

function T = FKinSpace(M, Slist, thetalist)
    % Forward Kinematics in Space Frame
    % Takes M: home configuration, Slist: screw axes in space frame,
    % thetalist: joint angles
    % Returns T: end-effector pose
    
    T = eye(4);
    for i = 1:length(thetalist)
        T = T * MatrixExp6(VecTose3(Slist(:, i) * thetalist(i)));
    end
    T = T * M;
end

function Js = JacobianSpace(Slist, thetalist)
    % Calculate Spatial Jacobian matrix
    % Takes Slist: screw axes in space frame, thetalist: joint angles
    % Returns Js: spatial Jacobian matrix
    
    n = length(thetalist);
    Js = zeros(6, n);
    T = eye(4);
    
    for i = 1:n
        % Current column is Adjoint(T) * Slist(:,i)
        Js(:, i) = Adjoint(T) * Slist(:, i);
        % Update T for next joint
        T = T * MatrixExp6(VecTose3(Slist(:, i) * thetalist(i)));
    end
end

function [thetalist, success] = IKinSpaceIterative(Slist, M, Tsd, thetalist0, eomg, ev)
% EXERCISE 3.3: Numerical Inverse Kinematics using Spatial Jacobian
% Uses the Newton-Raphson method with Spatial Jacobian
%
% Inputs:
%   Slist: Space screw axes list (6xn matrix)
%   M: Home configuration of end-effector (4x4 matrix)
%   Tsd: Desired end-effector configuration (4x4 matrix)
%   thetalist0: Initial guess for joint angles
%   eomg: Orientation error tolerance (epsilon)
%   ev: Position error tolerance (epsilon)
%
% Outputs:
%   thetalist: Joint angles solution
%   success: Logical true if converged, false otherwise
    
    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    
    % Calculate initial forward kinematics
    Tsb = FKinSpace(M, Slist, thetalist);
    
    % Calculate initial error twist in space frame
    Vs = se3ToVec(MatrixLog6(Tsb * TransInv(Tsd)));
    
    % Check convergence
    err = norm(Vs(1:3)) > eomg || norm(Vs(4:6)) > ev;
    
    while err && i < maxiterations
        i = i + 1;
        
        % Calculate spatial Jacobian
        Js = JacobianSpace(Slist, thetalist);
        
        % Update joint angles using pseudoinverse
        thetalist = thetalist + pinv(Js) * Vs;
        
        % Recalculate forward kinematics and error
        Tsb = FKinSpace(M, Slist, thetalist);
        Vs = se3ToVec(MatrixLog6(Tsb * TransInv(Tsd)));
        err = norm(Vs(1:3)) > eomg || norm(Vs(4:6)) > ev;
    end
    
    success = ~err;
    
    if success
        fprintf('IK Converged in %d iterations.\n', i);
    else
        fprintf('IK Failed to converge after %d iterations.\n', maxiterations);
    end
end

function Tinv = TransInv(T)
% TransInv: Computes the inverse of a homogeneous transformation matrix T
% Input: T - 4x4 homogeneous transformation matrix
% Output: Tinv - 4x4 inverse homogeneous transformation matrix

    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    Tinv = [R'    -R'*p;
           0 0 0 1];
end

function se3mat = VecTose3(V)
% VecTose3: Converts a 6x1 vector representation of a twist to a 4x4 se(3) matrix
% Input: V - 6x1 vector [omega; v] where omega is angular velocity, v is linear velocity
% Output: se3mat - 4x4 se(3) matrix representation

    omega = V(1:3);
    v = V(4:6);
    
    % Create skew-symmetric matrix from omega
    omgmat = [0     -omega(3)  omega(2);
              omega(3)  0     -omega(1);
             -omega(2)  omega(1)  0];
    
    se3mat = [omgmat  v;
              0 0 0 0];
end

function T = MatrixExp6(se3mat)
% MatrixExp6: Computes the matrix exponential of a se(3) matrix
% Input: se3mat - 4x4 se(3) matrix
% Output: T - 4x4 homogeneous transformation matrix (exponential map)

    omgmat = se3mat(1:3, 1:3);
    v = se3mat(1:3, 4);
    
    % Compute omega vector from skew-symmetric matrix
    omega = [omgmat(3,2); omgmat(1,3); omgmat(2,1)];
    theta = norm(omega);
    
    if theta < 1e-6
        % Small angle approximation
        R = eye(3) + omgmat;
        p = v;
    else
        % Rodrigues' formula for rotation matrix
        omgmat_hat = omgmat / theta;
        R = eye(3) + sin(theta)*omgmat_hat + (1-cos(theta))*omgmat_hat^2;
        
        % Position part using integral of rotation
        G = eye(3)*theta + (1-cos(theta))*omgmat_hat + (theta - sin(theta))*omgmat_hat^2;
        p = G*(v/theta);
    end
    
    T = [R p;
         0 0 0 1];
end

function Adj = Adjoint(T)
% Adjoint: Computes the adjoint transformation matrix for a homogeneous transformation T
% Input: T - 4x4 homogeneous transformation matrix
% Output: Adj - 6x6 adjoint transformation matrix

    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Create skew-symmetric matrix from p
    pmat = [0     -p(3)  p(2);
            p(3)  0     -p(1);
           -p(2)  p(1)  0];
    
    Adj = [R         zeros(3,3);
           pmat*R    R];
end

function se3mat = MatrixLog6(T)
% MatrixLog6: Computes the matrix logarithm of a homogeneous transformation matrix T
% Input: T - 4x4 homogeneous transformation matrix
% Output: se3mat - 4x4 se(3) matrix representation (logarithm map)

    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Compute rotation matrix logarithm
    omgmat = MatrixLog3(R);
    omega = [omgmat(3,2); omgmat(1,3); omgmat(2,1)];
    theta = norm(omega);
    
    if theta < 1e-6
        % Small angle approximation
        v = p;
    else
        % Compute G inverse for position part
        omgmat_hat = omgmat / theta;
        G = eye(3)*theta + (1-cos(theta))*omgmat_hat + (theta - sin(theta))*omgmat_hat^2;
        v = (G\p) * theta;
    end
    
    se3mat = [omgmat  v;
              0 0 0 0];
end

function Rlog = MatrixLog3(R)
% MatrixLog3: Computes the matrix logarithm of a rotation matrix R
% Input: R - 3x3 rotation matrix
% Output: Rlog - 3x3 skew-symmetric matrix (logarithm map)

    acosinput = (trace(R) - 1)/2;
    % Clamp to avoid numerical issues
    acosinput = max(min(acosinput, 1), -1);
    theta = acos(acosinput);
    
    if theta < 1e-6
        % Small angle approximation
        Rlog = (R - R')/2;
    else
        % Rodrigues' formula inverse
        Rlog = theta/(2*sin(theta))*(R - R');
    end
end

function V = se3ToVec(se3mat)
% se3ToVec: Converts a 4x4 se(3) matrix to a 6x1 vector representation
% Input: se3mat - 4x4 se(3) matrix
% Output: V - 6x1 vector [omega; v] where omega is angular velocity, v is linear velocity

    omega = [se3mat(3,2); se3mat(1,3); se3mat(2,1)];
    v = se3mat(1:3, 4);
    
    V = [omega; v];
end
