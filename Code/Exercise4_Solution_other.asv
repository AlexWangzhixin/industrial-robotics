clear; clc; close all;
% Adjust this path to point to your actual library folder
addpath('../Industrial_Robotics_Library'); 

%% --- Robot Definition (7-DOF) ---
% Define parameters based on visual inspection of Fig 4
L = 0.1; % Assumed link length [m]

% Define Home Configuration M (Vertical upright)
M = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, 7*L;
     0, 0, 0, 1];

% Define Screw Axes in Space Frame {s}
% Axis directions alternate: z, y, z, y, z, y, z (S-R-S)
% Origins are at [0;0; (i-1)*L]
S_list = zeros(6, 7);
S_list(:,1) = [0; 0; 1; cross([0;0;1], [0;0;0])];       % J1 (Base)
S_list(:,2) = [0; 1; 0; cross([0;1;0], [0;0;1*L])];     % J2
S_list(:,3) = [0; 0; 1; cross([0;0;1], [0;0;2*L])];     % J3
S_list(:,4) = [0; 1; 0; cross([0;1;0], [0;0;3*L])];     % J4
S_list(:,5) = [0; 0; 1; cross([0;0;1], [0;0;4*L])];     % J5
S_list(:,6) = [0; 1; 0; cross([0;1;0], [0;0;5*L])];     % J6
S_list(:,7) = [0; 0; 1; cross([0;0;1], [0;0;6*L])];     % J7

%% --- 4.1 Linear Motion with Numerical IK ---
fprintf('\n=== Exercise 4.1: Linear Motion ===\n');

% Initial Pose T_start (Given in assignment)
T_start = [0, 0, 1, 0.5;
           0, 1, 0, 0.1;
          -1, 0, 0, 0.2;
           0, 0, 0, 1];

% Linear Velocity specifications
v_lin = [-0.1; 0.0; 0.05]; % [m/s]
T_duration = 2.0;          % [s]
dt = 0.05;
time_steps = 0:dt:T_duration;
num_steps_lin = length(time_steps);

% 1. Find Initial Joint Config theta_start using Numerical IK
fprintf('Solving IK for initial configuration...\n');
theta_guess = [0; 0.5; 0; 0.5; 0; 0.5; 0]; % Non-zero guess to avoid singularity
theta_curr = IKinSpace(S_list, M, T_start, theta_guess, 1e-4, 100);

% 2. Simulation Loop
history_pos = zeros(3, num_steps_lin);
history_vel = zeros(7, num_steps_lin);
history_theta = zeros(7, num_steps_lin);

idx = 1;
for t = time_steps
    % Calculate desired pose at time t (Constant Orientation)
    % Position(t) = P_start + v * t
    p_t = T_start(1:3, 4) + v_lin * t;
    R_t = T_start(1:3, 1:3);
    T_desired = [R_t, p_t; 0, 0, 0, 1];
    
    % Solve IK for current step (use previous theta as guess)
    theta_curr = IKinSpace(S_list, M, T_desired, theta_curr, 1e-4, 50);
    
    % Store results
    history_pos(:, idx) = p_t;
    history_theta(:, idx) = theta_curr;
    
    % Calculate Joint Velocities: J * q_dot = V_spatial
    % We construct the spatial twist V_s. 
    % Since orientation is constant, w = 0. v = v_lin.
    V_s = [0;0;0; v_lin]; 
    
    Js = JacobianSpace(S_list, theta_curr);
    q_dot = pinv(Js) * V_s; % Pseudoinverse for redundant robot
    history_vel(:, idx) = q_dot;
    
    idx = idx + 1;
end

% Plotting 4.1
figure(1);
subplot(2,1,1);
plot3(history_pos(1,:), history_pos(2,:), history_pos(3,:), 'b.-'); grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
title('4.1 End-Effector Path (Linear Motion)');
axis equal;

subplot(2,1,2);
plot(time_steps, history_vel'); grid on;
xlabel('Time [s]'); ylabel('Joint Vel [rad/s]'); 
title('4.1 Joint Velocities');
legend('J1','J2','J3','J4','J5','J6','J7');

%% --- 4.2 & 4.3 Point-to-Point with Triangular Scaling ---
fprintf('\n=== Exercise 4.2 & 4.3: P2P with Triangular Scaling ===\n');

% Target Pose (Given at t=4s in assignment snippet)
T_end = [1, 0, 0, 0.17;
         0, 1, 0, 0.1;
         0, 0, 1, 0.2;
         0, 0, 0, 1];
Tf = 4.0; % Duration
N = 100;
t_vec = linspace(0, Tf, N);

% 1. Pre-calculate s(t) and v(t) for plotting and usage
s_vals = zeros(1, N);
sdot_vals = zeros(1, N);

for i = 1:N
    [s_vals(i), sdot_vals(i)] = TriangularScaling(Tf, t_vec(i));
end

traj_tri = CartesianTrajectory(T_start, T_end, Tf, N, '3');


% Plot Triangular Scaling Profile
s_vals_tri = zeros(1, N); sdot_vals_tri = zeros(1, N);
for i=1:N, [s_vals_tri(i), sdot_vals_tri(i)] = TriangularScaling(Tf, t_vec(i)); end

figure(2); 
subplot(2,1,1); plot(t_vec, s_vals_tri, 'LineWidth', 2); grid on; title('Triangular s(t)');
subplot(2,1,2); plot(t_vec, sdot_vals_tri, 'LineWidth', 2); grid on; title('Triangular v(t)');

% 2. Solve IK for Triangular Trajectory
fprintf('Solving IK for Triangular Trajectory...\n');
theta_curr = history_theta(:,1); % Reset to start configuration
hist_tri_theta = zeros(7, N);

% Pre-calculate constant Log term for interpolation: X_start^-1 * X_end
log_term = MatrixLog6(TransInv(T_start) * T_end);

for i = 1:N
    % Interpolate Pose: T(s) = T_start * exp(log_term * s)
    T_des = T_start * MatrixExp6(log_term * s_vals(i));
    
    % Solve IK
    theta_curr = IKinSpace(S_list, M, T_des, theta_curr, 1e-3, 20);
    hist_tri_theta(:,i) = theta_curr;
end

figure(3);
plot(t_vec, hist_tri_theta'); grid on;
title('Joint Angles for Triangular Profile'); 
xlabel('Time [s]'); ylabel('Angle [rad]');
legend('J1','J2','J3','J4','J5','J6','J7');

%% --- 4.4: Trapezoidal Scaling ---
fprintf('\n=== Exercise 4.4: Trapezoidal Scaling ===\n');

t_star = 1.0; % Rise time example (must be <= Tf/2)
s_trap = zeros(1, N);
sdot_trap = zeros(1, N);

% Calculate Trapezoidal Profile
for i = 1:N
    [s_trap(i), sdot_trap(i)] = TrapezoidalScaling(t_vec(i), Tf, t_star);
end

traj_tri = CartesianTrajectory(T_start, T_end, Tf, N, '4');

% Plot Trapezoidal Scaling Profile
figure(4);
subplot(2,1,1);
plot(t_vec, s_trap, 'LineWidth', 2); grid on; 
title(['Trapezoidal Scaling s(t) (t^*=' num2str(t_star) ')']); ylabel('s');
subplot(2,1,2);
plot(t_vec, sdot_trap, 'LineWidth', 2); grid on; 
title('Trapezoidal Scaling \dot{s}(t)'); xlabel('Time [s]'); ylabel('ds/dt');

% Optional: Verify Effect of t*
t_star_2 = 0.5;
sdot_trap_2 = zeros(1, N);
for i = 1:N
    [~, sdot_trap_2(i)] = TrapezoidalScaling(t_vec(i), Tf, t_star_2);
end
hold on; 
plot(t_vec, sdot_trap_2, '--r', 'LineWidth', 1.5); 
legend(['t^*=' num2str(t_star)], ['t^*=' num2str(t_star_2)]);

% %% --- Code Verification ---
% fprintf('\n=== Code Verification ===\n');
% % Verify IK solution for initial configuration
% [error_pos, error_rot] = VerifyKinematics(S_list, M, theta_guess, T_start);
% fprintf('Initial Config Error: Pos = %.2e m, Rot = %.2e rad\n', error_pos, error_rot);
% 
% % Verify IK solution for final configuration
% [error_pos_end, error_rot_end] = VerifyKinematics(S_list, M, hist_tri_theta(:,end), T_end);
% fprintf('Final Config Error: Pos = %.2e m, Rot = %.2e rad\n', error_pos_end, error_rot_end);
% 
% % Verify trajectory continuity
% max_jump = 0;
% for i = 2:length(hist_tri_theta)
%     jump = norm(hist_tri_theta(:,i) - hist_tri_theta(:,i-1));
%     max_jump = max(max_jump, jump);
% end
% fprintf('Max Joint Angle Jump between steps: %.2e rad\n', max_jump);
% 
% fprintf('\n--- Exercise 4 Complete ---\n');

%% --- my Functions ---

% 1. Triangular Scaling Function
function [s, sdot] = TriangularScaling(Tf, t)
    % Inputs: Tf = Total time, t = Current time
    % Outputs: s = Path parameter, sdot = Time derivative
    
    % Clamp t to [0, Tf]
    t = max(0, min(Tf, t));
    
    % Peak velocity v_max occurs at Tf/2
    % Area under velocity triangle must be 1.
    % 0.5 * base * height = 1 => 0.5 * Tf * v_max = 1 => v_max = 2/Tf
    v_max = 2 / Tf;
    
    if t <= Tf / 2
        % Acceleration Phase
        % v(t) = m*t, where m = v_max / (Tf/2) = 4/Tf^2
        sdot = (4 / Tf^2) * t;
        s = (2 / Tf^2) * t^2;
    else
        % Deceleration Phase
        % Symmetric to acceleration
        sdot = v_max - (4 / Tf^2) * (t - Tf/2);
        % s(t) = 1 - (distance remaining)
        time_left = Tf - t;
        s = 1 - (2 / Tf^2) * time_left^2;
    end
end

% 2. Trapezoidal Scaling Function
function [s, sdot] = TrapezoidalScaling(t, Tf, t_star)
    % Inputs: Tf = Total time, t_star = Rise time
    
    % Check validity
    if t_star > Tf/2
        error('t_star must be <= Tf/2');
    end
    t = max(0, min(Tf, t));
    
    % Calculate max velocity v_max
    % Area = v_max * (Tf - t_star) = 1
    v_max = 1 / (Tf - t_star);
    a = v_max / t_star; % Acceleration
    
    if t <= t_star
        % Phase 1: Acceleration
        s = 0.5 * a * t^2;
        sdot = a * t;
    elseif t <= (Tf - t_star)
        % Phase 2: Constant Velocity
        s_t_star = 0.5 * a * t_star^2; % distance at end of accel
        s = s_t_star + v_max * (t - t_star);
        sdot = v_max;
    else
        % Phase 3: Deceleration
        dt = Tf - t;
        s = 1 - 0.5 * a * dt^2;
        sdot = a * dt;
    end
end

% % 3. Verification Helper
% function [error_pos, error_rot] = VerifyKinematics(Slist, M, theta, T_desired)
%     T_actual = FKinSpace(M, Slist, theta);
% 
%     % Position error (Euclidean distance)
%     error_pos = norm(T_actual(1:3, 4) - T_desired(1:3, 4));
% 
%     % Rotation error (Magnitude of angular velocity vector needed to align frames)
%     R_err = T_actual(1:3, 1:3)' * T_desired(1:3, 1:3);
%     so3_err = MatrixLog3(R_err);
%     omega_err = [so3_err(3,2); so3_err(1,3); so3_err(2,1)];
%     error_rot = norm(omega_err);
% end