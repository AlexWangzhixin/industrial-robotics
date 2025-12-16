%% Industrial Robotics Assignment - Exercise 4
% Student: [Your Name]
% Date: 2025
% Description: Kinematics and Trajectory Planning (Using Extended CartesianTrajectory)
clear; clc; close all;
addpath('../Industrial_Robotics_Library'); 

%% --- Robot Definition (7-DOF) ---
L = 0.1; % Assumed link length [m]
M = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 7*L; 0, 0, 0, 1];

% S-R-S configuration
S_list = zeros(6, 7);
S_list(:,1) = [0; 0; 1; cross([0;0;1], [0;0;0])];
S_list(:,2) = [0; 1; 0; cross([0;1;0], [0;0;1*L])];
S_list(:,3) = [0; 0; 1; cross([0;0;1], [0;0;2*L])];
S_list(:,4) = [0; 1; 0; cross([0;1;0], [0;0;3*L])];
S_list(:,5) = [0; 0; 1; cross([0;0;1], [0;0;4*L])];
S_list(:,6) = [0; 1; 0; cross([0;1;0], [0;0;5*L])];
S_list(:,7) = [0; 0; 1; cross([0;0;1], [0;0;6*L])];

%% --- 4.1 Linear Motion (Straight Line) ---
fprintf('\n=== Exercise 4.1: Linear Motion ===\n');
T_start = [0, 0, 1, 0.5; 0, 1, 0, 0.1; -1, 0, 0, 0.2; 0, 0, 0, 1];
v_lin = [-0.1; 0.0; 0.05]; 
T_duration = 2.0; dt = 0.05;
time_steps = 0:dt:T_duration;

% Initial IK
theta_guess = [0; 0.5; 0; 0.5; 0; 0.5; 0];
theta_curr = CustomIK(S_list, M, T_start, theta_guess, 1e-4, 100);

% Simulation 4.1
hist_pos = []; hist_vel = [];
for t = time_steps
    p_t = T_start(1:3, 4) + v_lin * t;
    T_des = [T_start(1:3, 1:3), p_t; 0 0 0 1];
    theta_curr = CustomIK(S_list, M, T_des, theta_curr, 1e-4, 50);
    
    hist_pos = [hist_pos, p_t];
    Js = JacobianSpace(S_list, theta_curr);
    V_s = [0;0;0; v_lin];
    hist_vel = [hist_vel, pinv(Js)*V_s];
end

% Plot 4.1
figure(1); subplot(2,1,1); plot3(hist_pos(1,:), hist_pos(2,:), hist_pos(3,:), 'b.-'); 
grid on; axis equal; title('4.1 Linear Motion');
subplot(2,1,2); plot(time_steps, hist_vel'); grid on; title('4.1 Joint Velocities');

%% --- 4.2 & 4.3: Point-to-Point with Triangular Scaling ---
fprintf('\n=== Exercise 4.2 & 4.3: P2P with Triangular Scaling ===\n');
T_end = [1, 0, 0, 0.17; 0, 1, 0, 0.1; 0, 0, 1, 0.2; 0, 0, 0, 1];
Tf = 4.0; N = 100;
t_vec = linspace(0, Tf, N);

% === 关键修改：直接调用扩展的 CartesianTrajectory ===
% 方法设为 'Triangular'
traj_tri = CartesianTrajectory(T_start, T_end, Tf, N, 'Triangular');

% 提取 s(t) 用于绘图验证 (仅作展示，不用于IK循环)
s_vals_tri = zeros(1, N); sdot_vals_tri = zeros(1, N);
for i=1:N, [s_vals_tri(i), sdot_vals_tri(i)] = TriangularScaling(Tf, t_vec(i)); end

figure(2); 
subplot(2,1,1); plot(t_vec, s_vals_tri, 'LineWidth', 2); grid on; title('Triangular s(t)');
subplot(2,1,2); plot(t_vec, sdot_vals_tri, 'LineWidth', 2); grid on; title('Triangular v(t)');

% Solve IK for Triangular Trajectory
fprintf('Solving IK for Triangular Trajectory...\n');
theta_curr = theta_guess; % Reset
hist_tri_theta = zeros(7, N);

for i = 1:N
    % 直接从 CartesianTrajectory 生成的轨迹中获取第 i 个位姿
    T_des = traj_tri{i}; 
    theta_curr = CustomIK(S_list, M, T_des, theta_curr, 1e-3, 20);
    hist_tri_theta(:,i) = theta_curr;
end

figure(3); plot(t_vec, hist_tri_theta'); grid on; title('Joint Angles (Triangular)');

%% --- 4.4: Trapezoidal Scaling ---
fprintf('\n=== Exercise 4.4: Trapezoidal Scaling ===\n');
t_star = 1.0; % Rise time

% === 关键修改：调用带参数的 CartesianTrajectory ===
% 方法设为 'Trapezoidal'，并传递额外参数 t_star
traj_trap = CartesianTrajectory(T_start, T_end, Tf, N, 'Trapezoidal');

% 提取 s(t) 用于绘图
s_vals_trap = zeros(1, N); sdot_vals_trap = zeros(1, N);
for i=1:N, [s_vals_trap(i), sdot_vals_trap(i)] = TrapezoidalScaling(t_vec(i), Tf, t_star); end

figure(4);
subplot(2,1,1); plot(t_vec, s_vals_trap, 'LineWidth', 2); grid on; title(['Trapezoidal s(t), t^*=' num2str(t_star)]);
subplot(2,1,2); plot(t_vec, sdot_vals_trap, 'LineWidth', 2); grid on; title('Trapezoidal v(t)');

% 同样可以对 traj_trap 进行 IK 求解...
fprintf('\n--- Exercise 4 Complete ---\n');

%% === Helper Functions ===


% ---------------------------------------------------------
% 2. Scaling Functions (Implemented as local helpers)
% ---------------------------------------------------------
function [s, sdot] = TriangularScaling(Tf, t)
    t = max(0, min(Tf, t));
    v_max = 2 / Tf;
    if t <= Tf / 2
        sdot = (4 / Tf^2) * t;
        s = (2 / Tf^2) * t^2;
    else
        sdot = v_max - (4 / Tf^2) * (t - Tf/2);
        time_left = Tf - t;
        s = 1 - (2 / Tf^2) * time_left^2;
    end
end

function [s, sdot] = TrapezoidalScaling(t, Tf, t_star)
    if t_star > Tf/2, error('t_star must be <= Tf/2'); end
    t = max(0, min(Tf, t));
    v_max = 1 / (Tf - t_star);
    a = v_max / t_star;
    
    if t <= t_star
        s = 0.5 * a * t^2;
        sdot = a * t;
    elseif t <= (Tf - t_star)
        s_t_star = 0.5 * a * t_star^2;
        s = s_t_star + v_max * (t - t_star);
        sdot = v_max;
    else
        dt = Tf - t;
        s = 1 - 0.5 * a * dt^2;
        sdot = a * dt;
    end
end

% ---------------------------------------------------------
% 3. Standard Helpers (CustomIK, etc.)
% ---------------------------------------------------------
function theta = CustomIK(Slist, M, T_desired, theta_guess, tol, max_iter)
    theta = theta_guess;
    for i = 1:max_iter
        T_curr = FKinSpace(M, Slist, theta);
        T_diff = T_desired * TransInv(T_curr); 
        V_s = se3ToVec(MatrixLog6(T_diff));
        if norm(V_s) < tol, break; end
        Js = JacobianSpace(Slist, theta);
        theta = theta + pinv(Js) * V_s;
    end
end

function [R, p] = TransToRp(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
end

function T_inv = TransInv(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    T_inv = [R', -R'*p; 0, 0, 0, 1];
end

% Placeholder for standard scaling if not in library
function s = CubicTimeScaling(Tf, t)
    s = 3*(t/Tf)^2 - 2*(t/Tf)^3;
end
function s = QuinticTimeScaling(Tf, t)
    s = 10*(t/Tf)^3 - 15*(t/Tf)^4 + 6*(t/Tf)^5;
end