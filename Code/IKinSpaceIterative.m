function [thetalist, success] = IKinSpaceIterative(Slist, M, Tsd, thetalist0, eomg, ev)
% EXERCISE 3: Numerical Inverse Kinematics Implementation using Spatial Jacobian
% Uses the Newton-Raphson method with Spatial Jacobian for SCARA robot
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

    % Add library path
    addpath('../编程库/Industrial_Robotics_Library');

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20; % Set a reasonable limit
    
    % Step 2: Compute FK for initial guess
    Tsb = FKinSpace(M, Slist, thetalist);
    
    % Step 3: Compute Error Twist Vs in Space Frame
    % Vs = log(Tsb * Tsd^-1)
    Vs = se3ToVec(MatrixLog6(Tsb * TransInv(Tsd)));
    
    % Check convergence (Step 6 pre-check)
    err = norm(Vs(1:3)) > eomg || norm(Vs(4:6)) > ev;
    
    while err && i < maxiterations
        i = i + 1;
        
        % Step 4: Compute Spatial Jacobian
        Js = JacobianSpace(Slist, thetalist);
        
        % Step 5: Update Step using Pseudoinverse
        % theta = theta + pinv(Js) * Vs
        thetalist = thetalist + pinv(Js) * Vs;
        
        % Recalculate FK and Error for next iteration
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

function Js = JacobianSpace(Slist, thetalist)
% Calculate the Spatial Jacobian matrix
% Inputs:
%   Slist: Space screw axes list (6xn matrix)
%   thetalist: Current joint angles
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

function T = FKinSpace(M, Slist, thetalist)
% Forward Kinematics using Space Frame
% Inputs:
%   M: Home configuration (4x4 matrix)
%   Slist: Space screw axes list (6xn matrix)
%   thetalist: Joint angles
% Output:
%   T: End-effector pose (4x4 matrix)
    T = eye(4);
    for i = 1:length(thetalist)
        T = T * MatrixExp6(VecTose3(Slist(:, i) * thetalist(i)));
    end
    T = T * M;
end

function trajectory = generateTrajectory(theta_start, theta_end, t_total, N)
% Generate smooth joint space trajectory using cubic time scaling
% Inputs:
%   theta_start: Initial joint angles
%   theta_end: Target joint angles
%   t_total: Total time for trajectory
%   N: Number of trajectory points
% Output:
%   trajectory: Joint space trajectory (N x n matrix)
    t = linspace(0, t_total, N);
    s = cubicTimeScaling(t_total, t);
    
    % Generate trajectory for each joint
    n = length(theta_start);
    trajectory = zeros(N, n);
    for i = 1:n
        trajectory(:, i) = theta_start(i) + s * (theta_end(i) - theta_start(i));
    end
end

function s = cubicTimeScaling(t_total, t)
% Cubic time scaling function
% Inputs:
%   t_total: Total time
%   t: Current time(s) (scalar or vector)
% Output:
%   s: Time scaling factor (0 to 1)
    s = 3*(t/t_total).^2 - 2*(t/t_total).^3;
end

function s = quinticTimeScaling(t_total, t)
% Quintic time scaling function (optional for smoother trajectory)
% Inputs:
%   t_total: Total time
%   t: Current time(s) (scalar or vector)
% Output:
%   s: Time scaling factor (0 to 1)
    s = 10*(t/t_total).^3 - 15*(t/t_total).^4 + 6*(t/t_total).^5;
end