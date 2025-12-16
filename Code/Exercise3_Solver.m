%% Industrial Robotics Assignment - Exercise 3
% Description: Unitree Robot Leg Kinematics with Library Integration
% Requires: Industrial_Robotics_Library (JacobianSpace, etc.)

clear all; close all; clc;

% Add path to library if necessary
addpath('../Industrial_Robotics_Library/');

%% 1. Parameters & Setup
L1 = 0.3; L2 = 0.4; L0 = 0.4; 

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

% Parametric Trajectory Function (beta from pi to 0)
% Returns position relative to {F} origin
get_traj = @(beta) [ ...
    (x1+x2)/2 + a*cos(beta)*cos(w) - b*sin(beta)*sin(w); ...
    (y1+y2)/2 + a*cos(beta)*sin(w) + b*sin(beta)*cos(w); ...
    0 ]; 

% Plot trajectory shape
beta_plot = linspace(0, 2*pi, 100);
traj_points = zeros(3, length(beta_plot));
for i = 1:length(beta_plot)
    traj_points(:,i) = get_traj(beta_plot(i));
end

figure(1); 
plot(traj_points(1,:), traj_points(2,:), 'b.'); hold on;
plot(x1, y1, 'ko', 'MarkerFaceColor', 'k'); text(x1, y1-0.05, 'Start');
plot(x2, y2, 'ro', 'MarkerFaceColor', 'r'); text(x2, y2+0.05, 'End');
axis equal; grid on;
title('Exercise 3.1: Foot Trajectory (in Foot Frame)');
xlabel('x [m]'); ylabel('y [m]');


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


