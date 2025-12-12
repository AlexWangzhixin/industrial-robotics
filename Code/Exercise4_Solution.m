%% Industrial Robotics Assignment - Exercise 4
% Student: [Your Name]
% Date: 2025
% Description: Kinematics and Trajectory Planning for Hyper-Redundant Robot

clear; clc; close all;
% Add path to your library folder if necessary
% addpath('Industrial_Robotics_Library');

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

% Linear Velocity
v_lin = [-0.1; 0.0; 0.05]; % [m/s]
T_duration = 2.0;          % [s]
dt = 0.05;
time_steps = 0:dt:T_duration;

% 1. Find Initial Joint Config theta_start using Numerical IK
fprintf('Solving IK for initial configuration...\n');
theta_guess = [0; 0.5; 0; 0.5; 0; 0.5; 0]; % Non-zero guess to avoid singularity
theta_curr = CustomIK(S_list, M, T_start, theta_guess, 1e-4, 100);

% 2. Simulation Loop
history_pos = [];
history_vel = [];
history_theta = [];

for t = time_steps
    % Calculate desired pose at time t (Constant Orientation)
    % Position(t) = P_start + v * t
    p_t = T_start(1:3, 4) + v_lin * t;
    R_t = T_start(1:3, 1:3);
    T_desired = [R_t, p_t; 0, 0, 0, 1];
    
    % Solve IK for current step (use previous theta as guess)
    theta_curr = CustomIK(S_list, M, T_desired, theta_curr, 1e-4, 50);
    
    % Store results
    history_pos = [history_pos, p_t];
    history_theta = [history_theta, theta_curr];
    
    % Calculate Joint Velocities: J * q_dot = V_spatial
    % V_spatial = [0; v_lin] (approximately, since R is constant, w=0)
    Js = JacobianSpace(S_list, theta_curr);
    V_s = [0;0;0; v_lin]; % Twist in space (w=0)
    q_dot = pinv(Js) * V_s;
    history_vel = [history_vel, q_dot];
end

% Plotting 4.1
figure(1);
subplot(2,1,1);
plot3(history_pos(1,:), history_pos(2,:), history_pos(3,:), 'b.-'); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z'); title('4.1 End-Effector Path');
axis equal;

subplot(2,1,2);
plot(time_steps, history_vel'); grid on;
xlabel('Time [s]'); ylabel('Joint Vel [rad/s]'); title('4.1 Joint Velocities');

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

% Generate Trajectory using Triangular Scaling
traj_tri = GenerateTrajectory(T_start, T_end, Tf, N, 'Triangular');

% Plot Triangular Scaling Profile (s(t) and s_dot(t))
figure(2);
subplot(2,1,1);
plot(t_vec, traj_tri.s); grid on; title('Triangular Scaling: s(t)');
subplot(2,1,2);
plot(t_vec, traj_tri.sdot); grid on; title('Triangular Scaling: v(t)'); xlabel('Time [s]');

% Solve IK for Triangular Trajectory
fprintf('Solving IK for Triangular Trajectory...\n');
theta_curr = history_theta(:,1); % Reset to start
hist_tri_theta = zeros(7, N);

for i = 1:N
    T_des = traj_tri.T{i};
    theta_curr = CustomIK(S_list, M, T_des, theta_curr, 1e-3, 20);
    hist_tri_theta(:,i) = theta_curr;
end

figure(3);
plot(t_vec, hist_tri_theta'); grid on;
title('Joint Angles for Triangular Profile'); xlabel('Time [s]'); ylabel('Rad');

%% --- 4.4: Trapezoidal Scaling ---
fprintf('\n=== Exercise 4.4: Trapezoidal Scaling ===\n');

t_star = 0.5; % Rise time example
traj_trap = GenerateTrajectory(T_start, T_end, Tf, N, 'Trapezoidal', t_star);

% Plot Trapezoidal Scaling Profile
figure(4);
subplot(2,1,1);
plot(t_vec, traj_trap.s, 'LineWidth', 2); grid on; 
title(['Trapezoidal Scaling s(t) (t^*=' num2str(t_star) ')']);
subplot(2,1,2);
plot(t_vec, traj_trap.sdot, 'LineWidth', 2); grid on; 
title('Trapezoidal Scaling v(t)'); xlabel('Time [s]');

% Compare with different t*
t_star_2 = 1.5;
traj_trap_2 = GenerateTrajectory(T_start, T_end, Tf, N, 'Trapezoidal', t_star_2);
hold on; plot(t_vec, traj_trap_2.sdot, '--r'); 
legend(['t^*=' num2str(t_star)], ['t^*=' num2str(t_star_2)]);

%% --- Code Verification ---
fprintf('\n=== Code Verification ===\n');

% Verify IK solution for initial configuration
[error_pos, error_rot] = VerifyKinematics(S_list, M, theta_guess, T_start);
fprintf('Initial Configuration: Position Error = %.2e m, Rotation Error = %.2e rad\n', error_pos, error_rot);

% Verify IK solution for final configuration
[error_pos_end, error_rot_end] = VerifyKinematics(S_list, M, hist_tri_theta(:,end), T_end);
fprintf('Final Configuration: Position Error = %.2e m, Rotation Error = %.2e rad\n', error_pos_end, error_rot_end);

% Verify trajectory continuity
max_jump = 0;
for i = 2:length(hist_tri_theta)
    jump = norm(hist_tri_theta(:,i) - hist_tri_theta(:,i-1));
    max_jump = max(max_jump, jump);
end
fprintf('Maximum Joint Angle Jump: %.2e rad\n', max_jump);

%% --- Additional Visualizations ---
fprintf('\n=== Additional Visualizations ===\n');

% Plot Joint Accelerations for Linear Motion
dt_lin = dt;
PlotJointAccelerations(time_steps, history_vel, dt_lin);

% Plot End-Effector Velocity and Acceleration for Linear Motion
PlotEndEffectorVelocityAcceleration(time_steps, history_pos, dt_lin);

% Compare Different Scaling Methods
CompareScalingMethods(Tf, N);

% Plot Joint Acceleration Distribution for Triangular Trajectory
% Calculate joint accelerations for triangular trajectory
n_joints = size(hist_tri_theta, 1);
vel_tri_history = zeros(n_joints, N);
dt_tri = Tf/(N-1);

for i = 2:N
    vel_tri_history(:, i) = (hist_tri_theta(:, i) - hist_tri_theta(:, i-1)) / dt_tri;
end
vel_tri_history(:, 1) = vel_tri_history(:, 2); % Extend first value

PlotJointAccelerations(t_vec, vel_tri_history, dt_tri);

fprintf('\n--- Exercise 4 Complete with Enhancements ---
');

%% --- Helper Functions ---

function traj = GenerateTrajectory(Xstart, Xend, Tf, N, method, varargin)
    time = linspace(0, Tf, N);
    traj.s = zeros(1, N);
    traj.sdot = zeros(1, N);
    traj.T = cell(1, N);
    
    for i = 1:N
        t = time(i);
        if strcmp(method, 'Triangular')
            [s, sdot] = TriangularScaling(t, Tf);
        elseif strcmp(method, 'Trapezoidal')
            t_star = varargin{1};
            [s, sdot] = TrapezoidalScaling(t, Tf, t_star);
        else % Default Cubic
             % (Simple cubic implementation omitted for brevity, focusing on requested types)
             s = (3*(t/Tf)^2 - 2*(t/Tf)^3);
             sdot = 0; 
        end
        
        traj.s(i) = s;
        traj.sdot(i) = sdot;
        
        % Interpolate Pose (using Matrix Logarithm for rotation)
        % X(s) = Xstart * exp( log(Xstart^-1 * Xend) * s )
        X_diff = Xstart \ Xend;
        twist_se3 = MatrixLog6(X_diff);
        traj.T{i} = Xstart * MatrixExp6(twist_se3 * s);
    end
end

function [s, sdot] = TriangularScaling(t, Tf)
    % Peak velocity
    v_max = 2 / Tf;
    
    if t <= Tf/2
        % Acceleration phase
        a = v_max / (Tf/2);
        s = 0.5 * a * t^2;
        sdot = a * t;
    else
        % Deceleration phase
        % s(t) = 1 - (distance from end)
        dt = Tf - t;
        a = v_max / (Tf/2);
        s = 1 - 0.5 * a * dt^2;
        sdot = a * dt;
    end
end

function [s, sdot] = TrapezoidalScaling(t, Tf, t_star)
    % Check validity
    if t_star > Tf/2
        error('t_star must be <= Tf/2');
    end
    
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

function theta = CustomIK(Slist, M, T_desired, theta_guess, tol, max_iter)
    % Newton-Raphson IK Solver (without IKinSpace)
    theta = theta_guess;
    for i = 1:max_iter
        % Forward Kinematics
        T_curr = M;
        for j = 1:length(theta)
            T_curr = T_curr * MatrixExp6(VecTose3(Slist(:,j)*theta(j))); % Note: Simplified FK for loop
        end
        % Actually, better use Product of Exponentials formula properly:
        % T = e^(S1t1) * ... * e^(Sntn) * M
        % Correct implementation:
        T_curr = FKinSpace(M, Slist, theta);
        
        % Error Twist in Space Frame? Or Body?
        % Using Body Frame convergence is often numerically more stable or Space
        % Let's use Space Frame formulation: V_s = J_s * q_dot
        % log(T_curr^-1 * T_des) is body twist.
        % log(T_des * T_curr^-1) is space twist.
        
        T_err_matrix = T_desired / T_curr; % T_des * inv(T_curr)
        V_s_se3 = MatrixLog6(T_err_matrix);
        V_s = se3ToVec(V_s_se3);
        
        if norm(V_s) < tol
            break;
        end
        
        J_s = JacobianSpace(Slist, theta);
        % Pseudoinverse for redundant manipulator
        theta = theta + pinv(J_s) * V_s;
    end
end

% --- Standard Library Functions Implementation ---

function T = FKinSpace(M, Slist, thetalist)
% FKinSpace: Computes forward kinematics using Product of Exponentials formula
% Inputs:
%   M: Home configuration matrix (4x4)
%   Slist: Space screw axes list (6xn matrix)
%   thetalist: Joint angles (nx1 vector)
% Output:
%   T: End-effector pose (4x4 matrix)
    
    T = eye(4);
    for i = 1:length(thetalist)
        T = T * MatrixExp6(VecTose3(Slist(:, i) * thetalist(i)));
    end
    T = T * M;
end

function Js = JacobianSpace(Slist, thetalist)
% JacobianSpace: Computes the spatial Jacobian matrix
% Inputs:
%   Slist: Space screw axes list (6xn matrix)
%   thetalist: Joint angles (nx1 vector)
% Output:
%   Js: Spatial Jacobian matrix (6xn)
    
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

function se3mat = MatrixLog6(T)
% MatrixLog6: Computes the matrix logarithm of a homogeneous transformation matrix
% Input:
%   T: Homogeneous transformation matrix (4x4)
% Output:
%   se3mat: se(3) matrix representation (4x4)
    
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
% MatrixLog3: Computes the matrix logarithm of a rotation matrix
% Input:
%   R: Rotation matrix (3x3)
% Output:
%   Rlog: Skew-symmetric matrix (3x3)
    
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

function T = MatrixExp6(se3mat)
% MatrixExp6: Computes the matrix exponential of a se(3) matrix
% Input:
%   se3mat: se(3) matrix (4x4)
% Output:
%   T: Homogeneous transformation matrix (4x4)
    
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

function se3mat = VecTose3(V)
% VecTose3: Converts a 6x1 vector representation of a twist to a 4x4 se(3) matrix
% Input:
%   V: 6x1 vector [omega; v] where omega is angular velocity, v is linear velocity
% Output:
%   se3mat: 4x4 se(3) matrix representation
    
    omega = V(1:3);
    v = V(4:6);
    
    % Create skew-symmetric matrix from omega
    omgmat = [0     -omega(3)  omega(2);
              omega(3)  0     -omega(1);
             -omega(2)  omega(1)  0];
    
    se3mat = [omgmat  v;
              0 0 0 0];
end

function V = se3ToVec(se3mat)
% se3ToVec: Converts a 4x4 se(3) matrix to a 6x1 vector representation
% Input:
%   se3mat: 4x4 se(3) matrix
% Output:
%   V: 6x1 vector [omega; v] where omega is angular velocity, v is linear velocity
    
    omega = [se3mat(3,2); se3mat(1,3); se3mat(2,1)];
    v = se3mat(1:3, 4);
    
    V = [omega; v];
end

function Tinv = TransInv(T)
% TransInv: Computes the inverse of a homogeneous transformation matrix
% Input:
%   T: 4x4 homogeneous transformation matrix
% Output:
%   Tinv: 4x4 inverse homogeneous transformation matrix
    
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    Tinv = [R'    -R'*p;
           0 0 0 1];
end

function Adj = Adjoint(T)
% Adjoint: Computes the adjoint transformation matrix for a homogeneous transformation T
% Input:
%   T: 4x4 homogeneous transformation matrix
% Output:
%   Adj: 6x6 adjoint transformation matrix
    
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Create skew-symmetric matrix from p
    pmat = [0     -p(3)  p(2);
            p(3)  0     -p(1);
           -p(2)  p(1)  0];
    
    Adj = [R         zeros(3,3);
           pmat*R    R];
end

% --- Code Verification Functions ---
function [error_pos, error_rot] = VerifyKinematics(Slist, M, theta, T_desired)
% VerifyKinematics: Verifies the consistency between FK and IK
% Inputs:
%   Slist: Space screw axes list (6xn matrix)
%   M: Home configuration matrix (4x4)
%   theta: Joint angles (nx1 vector)
%   T_desired: Desired end-effector pose (4x4 matrix)
% Outputs:
%   error_pos: Position error (Euclidean norm)
%   error_rot: Rotation error (angular norm)
    
    % Compute forward kinematics
    T_actual = FKinSpace(M, Slist, theta);
    
    % Position error
    error_pos = norm(T_actual(1:3, 4) - T_desired(1:3, 4));
    
    % Rotation error (using axis-angle representation)
    R_err = T_actual(1:3, 1:3)' * T_desired(1:3, 1:3);
    omega_err = MatrixLog3(R_err);
    error_rot = norm([omega_err(3,2); omega_err(1,3); omega_err(2,1)]);
end

% --- Additional Visualization Functions ---
function PlotJointAccelerations(time, vel_history, dt)
% PlotJointAccelerations: Plots joint accelerations
% Inputs:
%   time: Time vector
%   vel_history: Joint velocity history
%   dt: Time step
    
    % Calculate accelerations using finite differences
    n_joints = size(vel_history, 1);
    n_steps = size(vel_history, 2);
    acc_history = zeros(n_joints, n_steps);
    
    for i = 2:n_steps
        acc_history(:, i) = (vel_history(:, i) - vel_history(:, i-1)) / dt;
    end
    acc_history(:, 1) = acc_history(:, 2); % Extend first value
    
    figure;
    plot(time, acc_history');
    grid on;
    title('Joint Accelerations vs Time');
    xlabel('Time [s]');
    ylabel('Joint Acceleration [rad/s²]');
    legend(sprintf('Joint %d', 1:n_joints));
    
    % Plot acceleration histogram
    figure;
    histogram(acc_history', 'Normalization', 'probability');
    grid on;
    title('Joint Acceleration Distribution');
    xlabel('Joint Acceleration [rad/s²]');
    ylabel('Probability');
    legend(sprintf('Joint %d', 1:n_joints));
end

function PlotEndEffectorVelocityAcceleration(time, pos_history, dt)
% PlotEndEffectorVelocityAcceleration: Plots end-effector velocity and acceleration
% Inputs:
%   time: Time vector
%   pos_history: End-effector position history
%   dt: Time step
    
    % Calculate velocities and accelerations using finite differences
    n_steps = size(pos_history, 2);
    vel_history = zeros(3, n_steps);
    acc_history = zeros(3, n_steps);
    
    for i = 2:n_steps
        vel_history(:, i) = (pos_history(:, i) - pos_history(:, i-1)) / dt;
    end
    vel_history(:, 1) = vel_history(:, 2); % Extend first value
    
    for i = 2:n_steps
        acc_history(:, i) = (vel_history(:, i) - vel_history(:, i-1)) / dt;
    end
    acc_history(:, 1) = acc_history(:, 2); % Extend first value
    
    % Plot velocity
    figure;
    subplot(2,1,1);
    plot(time, vel_history');
    grid on;
    title('End-Effector Velocity vs Time');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    legend('vx', 'vy', 'vz');
    
    % Plot acceleration
    subplot(2,1,2);
    plot(time, acc_history');
    grid on;
    title('End-Effector Acceleration vs Time');
    xlabel('Time [s]');
    ylabel('Acceleration [m/s²]');
    legend('ax', 'ay', 'az');
end

function CompareScalingMethods(Tf, N)
% CompareScalingMethods: Compares different scaling methods
% Inputs:
%   Tf: Total time
%   N: Number of samples
    
    time = linspace(0, Tf, N);
    
    % Calculate scaling functions
    s_tri = zeros(1, N);
    sdot_tri = zeros(1, N);
    s_trap1 = zeros(1, N);
    sdot_trap1 = zeros(1, N);
    s_trap2 = zeros(1, N);
    sdot_trap2 = zeros(1, N);
    
    for i = 1:N
        t = time(i);
        [s_tri(i), sdot_tri(i)] = TriangularScaling(t, Tf);
        [s_trap1(i), sdot_trap1(i)] = TrapezoidalScaling(t, Tf, Tf/4);
        [s_trap2(i), sdot_trap2(i)] = TrapezoidalScaling(t, Tf, Tf/8);
    end
    
    % Plot scaling functions
    figure;
    subplot(2,1,1);
    plot(time, s_tri, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, s_trap1, 'r--', 'LineWidth', 2);
    plot(time, s_trap2, 'g-.', 'LineWidth', 2);
    grid on;
    title('Scaling Functions Comparison');
    xlabel('Time [s]');
    ylabel('s(t)');
    legend('Triangular', 'Trapezoidal (t*=Tf/4)', 'Trapezoidal (t*=Tf/8)');
    
    % Plot velocity profiles
    subplot(2,1,2);
    plot(time, sdot_tri, 'b-', 'LineWidth', 2);
    hold on;
    plot(time, sdot_trap1, 'r--', 'LineWidth', 2);
    plot(time, sdot_trap2, 'g-.', 'LineWidth', 2);
    grid on;
    title('Velocity Profiles Comparison');
    xlabel('Time [s]');
    ylabel('v(t)');
    legend('Triangular', 'Trapezoidal (t*=Tf/4)', 'Trapezoidal (t*=Tf/8)');
end