%% Industrial Robotics Assignment - Exercise 5
% Student Name: [Your Name]
% Date: 2025-12-12
% Description: Kinematics, Jacobian, IK, and Trajectory for 6-DOF Humanoid Arm

clear; clc; close all;
addpath('../Industrial_Robotics_Library');

%% --- 1. Robot Definition (Exercise 5.1) ---
fprintf('=== Exercise 5.1: Robot Model & Workspace ===\n');

% Link Lengths [m]
L1 = 0.15; L2 = 0.1; L3 = 0.25; L4 = 0.2;

% Home Configuration M (Vertical Upright)
M = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, L1+L2+L3+L4;
     0, 0, 0, 1];

% Screw Axes in Space Frame {s}
% Axis origins
q1=[0;0;0]; q2=[0;0;L1]; q3=[0;0;L1+L2];
q_wrist = [0;0;L1+L2+L3];

% Axis directions (Assumed standard Anthropomorphic: Z, Y, Y, Z, Y, X)
w1=[0;0;1]; w2=[0;1;0]; w3=[0;1;0];
w4=[0;0;1]; w5=[0;1;0]; w6=[1;0;0];

% Construct Slist
Slist = [ [w1; -cross(w1,q1)], ...
          [w2; -cross(w2,q2)], ...
          [w3; -cross(w3,q3)], ...
          [w4; -cross(w4,q_wrist)], ...
          [w5; -cross(w5,q_wrist)], ...
          [w6; -cross(w6,q_wrist)] ];

fprintf('Screw Axes Slist:\n'); disp(Slist);
fprintf('Home Configuration M:\n'); disp(M);

% Workspace Plotting
figure('Name', 'Ex 5.1: Workspace', 'Color', 'w');
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Workspace (J1, J2, J3 limits)');

N_samples = 1000;
% Limits: q1[-90,90], q2[-70,90], q3[-90,90] (others free, assume 0 for WS shape)
for i = 1:N_samples
    t1 = deg2rad(-90 + 180*rand());
    t2 = deg2rad(-70 + 160*rand());
    t3 = deg2rad(-90 + 180*rand());
    t4 = 0; t5 = 0; t6 = 0; % Keep wrist fixed for arm workspace volume
    
    theta = [t1; t2; t3; t4; t5; t6];
    T = FKinSpace(M, Slist, theta);
    plot3(T(1,4), T(2,4), T(3,4), 'b.', 'MarkerSize', 5);
end
% Plot robot at Home
draw_robot(M, Slist, zeros(6,1), 'k', 2);

%% --- 2. Jacobian Analysis (Exercise 5.2) ---
fprintf('\n=== Exercise 5.2: Jacobian Calculation ===\n');

theta_test_deg = [50, 10, -15, 30, 30, -15];
theta_test = deg2rad(theta_test_deg)';
vel_test_deg = [10, 80, -50, 15, 20, -50];
theta_dot_test = deg2rad(vel_test_deg)';

% Custom Jacobian Calculation (Without JacobianSpace)
Js = custom_Jacobian(Slist, theta_test);

fprintf('Calculated Jacobian Js at test config:\n'); disp(Js);

% Plot Jacobian Columns (Linear Velocity Vectors)
figure('Name', 'Ex 5.2: Jacobian Vectors', 'Color', 'w');
hold on; grid on; axis equal; view(3);
title('Jacobian Linear Velocity Columns');
T_curr = FKinSpace(M, Slist, theta_test);
p_ee = T_curr(1:3,4);
draw_robot(M, Slist, theta_test, 'k', 0.5);
% Plot vectors
colors = lines(6);
for i = 1:6
    v_lin = Js(4:6, i); % Linear part
    quiver3(p_ee(1), p_ee(2), p_ee(3), v_lin(1), v_lin(2), v_lin(3), 0.2, 'Color', colors(i,:), 'LineWidth', 2);
end

% Compute End-Effector Velocity
V_spatial = Js * theta_dot_test;
fprintf('End-Effector Spatial Twist (w, v):\n'); disp(V_spatial);

%% --- 3. Inverse Kinematics (Exercise 5.3) ---
fprintf('\n=== Exercise 5.3: Inverse Kinematics ===\n');

% Target 1: x_e
pos_1 = [0.2; 0.1; 0.3];
eul_1 = deg2rad([10, 30, 20]); % Assumed XYZ or similar
R_1 = eul2r_custom(eul_1(1), eul_1(2), eul_1(3)); % Helper function
T_1 = [R_1, pos_1; 0 0 0 1];

% Target 2: T_se
T_2 = [0, -1, 0, 0.2;
       1,  0, 0, 0.3;
       0,  0, 1, 0.2;
       0,  0, 0, 1];

fprintf('Solving IK for Target 1...\n');
theta_sol_1 = custom_IK(Slist, M, T_1, zeros(6,1));
fprintf('Solution 1 (deg): %s\n', mat2str(rad2deg(theta_sol_1)', 4));

fprintf('Solving IK for Target 2...\n');
theta_sol_2 = custom_IK(Slist, M, T_2, theta_sol_1); % Use prev as guess
fprintf('Solution 2 (deg): %s\n', mat2str(rad2deg(theta_sol_2)', 4));

%% --- 4. Point-to-Point Trajectory (Exercise 5.4) ---
fprintf('\n=== Exercise 5.4: P2P Trajectory ===\n');

Tf = 0.5;
dt = 0.025;
steps = Tf/dt;
traj = CartesianTrajectory(T_1, T_2, Tf, steps, 3); % Cubic

history_q = [];
history_v = [];
theta_curr = theta_sol_1;

for i = 1:length(traj)
    T_des = traj{i};
    % Use library IKinSpace here as allowed
    theta_curr = IKinSpace(Slist, M, T_des, theta_curr, 1e-4, 1e-4);
    history_q = [history_q, theta_curr];
end

% Calc velocities
history_v = diff(history_q, 1, 2) / dt;
max_v_deg = max(max(abs(rad2deg(history_v))));

fprintf('Trajectory T_f = %.2f s. Max Joint Velocity = %.2f deg/s\n', Tf, max_v_deg);
limit_v = 30;
if max_v_deg > limit_v
    req_Tf = Tf * (max_v_deg / limit_v);
    fprintf('Constraint Violated! Required T_f approx %.2f s\n', req_Tf);
end

% Plotting 5.4
figure('Name', 'Ex 5.4: Joint Velocities');
plot(rad2deg(history_v')); grid on;
yline(30, 'r--'); yline(-30, 'r--');
title('Joint Velocities (Tf=0.5s)'); ylabel('deg/s');

%% --- 5. Multi-Waypoint Trajectory (Exercise 5.5) ---
fprintf('\n=== Exercise 5.5: Multi-Waypoint Trajectory ===\n');

% Waypoints: Home -> T1 -> T2 -> Home
T_home = FKinSpace(M, Slist, zeros(6,1));
waypoints = {T_home, T_1, T_2, T_home};
Tf_total = 25.0;
N_points = 4;
Tf_segment = Tf_total / (N_points - 1);
N_steps_seg = 33; % Approx 100 total

history_q_multi = [];
history_pos = [];
theta_curr = zeros(6,1);

for i = 1:3
    % Generate segment
    traj_seg = CartesianTrajectory(waypoints{i}, waypoints{i+1}, Tf_segment, N_steps_seg, 5);
    for j = 1:length(traj_seg)
        T_des = traj_seg{j};
        theta_curr = IKinSpace(Slist, M, T_des, theta_curr, 1e-4, 1e-4);
        history_q_multi = [history_q_multi, theta_curr];
        history_pos = [history_pos, T_des(1:3,4)];
    end
end

figure('Name', 'Ex 5.5: Multi-Waypoint Results');
subplot(2,1,1);
plot(history_pos'); legend('x','y','z'); grid on; title('End-Effector Position');
subplot(2,1,2);
plot(rad2deg(history_q_multi')); grid on; title('Joint Angles');

%% --- Helper Functions ---

function Js = custom_Jacobian(Slist, theta)
    % Manual implementation of Space Jacobian
    % J = [S1, Ad(e^S1t1)*S2, ...]
    n = size(Slist, 2);
    Js = zeros(6, n);
    T = eye(4);
    Js(:,1) = Slist(:,1);
    for i = 2:n
        T = T * MatrixExp6(VecTose3(Slist(:,i-1) * theta(i-1)));
        Js(:,i) = Adjoint(T) * Slist(:,i);
    end
end

function theta = custom_IK(Slist, M, T_sd, theta_guess)
    % Newton-Raphson IK
    theta = theta_guess;
    tol = 1e-4;
    max_iter = 50;
    for i = 1:max_iter
        T_curr = FKinSpace(M, Slist, theta);
        V_b = MatrixLog6(TransInv(T_curr) * T_sd);
        V_b_vec = se3ToVec(V_b);
        if norm(V_b_vec) < tol
            break;
        end
        J_b = JacobianBody(Slist, theta); % Using library for Jacobian Body or convert Js
        % Or use Space Jacobian relationship: Vs = Ad(T)*Vb -> J_s*qdot = Ad(T)*Vb
        % Actually simpler: J_s * dtheta = V_s. 
        % V_s = MatrixLog6(T_sd * inv(T_curr)).
        % Let's stick to Space Jacobian as we have custom_Jacobian
        V_s_se3 = MatrixLog6(T_sd * TransInv(T_curr));
        V_s_vec = se3ToVec(V_s_se3);
        
        Js = custom_Jacobian(Slist, theta);
        theta = theta + pinv(Js) * V_s_vec;
    end
end

function R = eul2r_custom(phi, theta, psi)
    % ZYZ Euler to Rotation (Standard Industrial) or XYZ
    % Assuming XYZ fixed for generic 'Euler Angles' unless specified
    R = rotz(phi) * roty(theta) * rotx(psi); % ZYX Sequence example
    % Note: Replace with specific convention if defined in course materials
end

function draw_robot(M, Slist, theta, col, width)
    % Simple stick figure drawing
    points = zeros(3, 7);
    T = eye(4);
    points(:,1) = T(1:3,4);
    for i = 1:6
        T = T * MatrixExp6(VecTose3(Slist(:,i)*theta(i)));
        points(:,i+1) = T(1:3,4);
    end
    % Add End-Effector offset M relative to last joint? 
    % M is usually absolute home.
    % T_final = FKinSpace(M,Slist,theta);
    % points(:,8) = T_final(1:3,4);
    
    plot3(points(1,:), points(2,:), points(3,:), [col '-o'], 'LineWidth', width);
end

% Library Standard Functions (Minimal local definitions if library missing)
% Assumes Modern Robotics Library functions (MatrixExp6, VecTose3, Adjoint, etc.) are available.